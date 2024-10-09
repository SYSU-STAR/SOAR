
#include <heterogeneous_manager/heterogeneous_planner_manager.h>
#include <traj_utils/planning_visualization.h>

#include <heterogeneous_manager/heterogeneous_planner_fsm.h>
#include <heterogeneous_manager/hetero_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace hetero_planner {
void HeterogeneousPlannerFSM::init(ros::NodeHandle& nh)
{
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

  /* Initialize main modules */
  hetero_manager_.reset(new HeterogenousPlannerManager);
  hetero_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  state_ = FSM_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  resolution_ = hetero_manager_->getResolution();
  drone_num_ = (int)hetero_manager_->hd_->swarm_state_.size();

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &HeterogeneousPlannerFSM::FSMCallback, this);
  safety_timer_ =
      nh.createTimer(ros::Duration(0.02), &HeterogeneousPlannerFSM::safetyCallback, this);
  frontier_timer_ =
      nh.createTimer(ros::Duration(0.5), &HeterogeneousPlannerFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/move_base_simple/goal", 1, &HeterogeneousPlannerFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &HeterogeneousPlannerFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  minco_traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("/planning/minco_traj", 10);

  /* Multi-robot related */
  drone_state_timer_ =
      nh.createTimer(ros::Duration(0.05), &HeterogeneousPlannerFSM::droneStateTimerCallback, this);
  drone_state_pub_ =
      nh.advertise<heterogeneous_manager::DroneState>("/hetero_swarm/drone_state_send", 10);
  drone_state_sub_ = nh.subscribe(
      "/hetero_swarm/drone_state_recv", 10, &HeterogeneousPlannerFSM::droneStateMsgCallback, this);

  task_assignment_timer_ = nh.createTimer(
      ros::Duration(0.1), &HeterogeneousPlannerFSM::taskAssignmentTimerCallback, this);
  task_assignment_pub_ =
      nh.advertise<heterogeneous_manager::ViewpointsTask>("/hetero_swarm/task_assignment_send", 10);
  task_assignment_sub_ = nh.subscribe("/hetero_swarm/task_assignment_recv", 10,
      &HeterogeneousPlannerFSM::taskAssignmentMsgCallback, this);

  visited_tour_timer_ =
      nh.createTimer(ros::Duration(0.05), &HeterogeneousPlannerFSM::visitedTourTimerCallback, this);
  visited_tour_pub_ =
      nh.advertise<heterogeneous_manager::VisitedTour>("/hetero_swarm/visited_tour_send", 10);
  visited_tour_sub_ = nh.subscribe("/hetero_swarm/visited_tour_recv", 10,
      &HeterogeneousPlannerFSM::visitedTourMsgCallback, this);
}

void HeterogeneousPlannerFSM::FSMCallback(const ros::TimerEvent& e)
{
  ROS_INFO_STREAM_THROTTLE(
      1.0, "drone_" << getSlfDroneId() << "[FSM]: state: " << fd_->state_str_[int(state_)]);

  if (isExplorer()) {
    switch (state_) {

      case INIT: {
        if (!fd_->have_odom_ || !hetero_manager_->isHaveMap()) {
          ROS_WARN_THROTTLE(1.0, "drone_%d wait for odometry or grid_map ready", getSlfDroneId());
          return;
        }
        // Go to wait trigger when odom is ok
        transitState(WAIT_TRIGGER, "FSM");
        break;
      }

      case WAIT_TRIGGER: {
        // Do nothing but wait for trigger
        ROS_WARN_THROTTLE(1.0, "drone_%d wait for trigger.", getSlfDroneId());
        break;
      }

      case FINISH: {
        ROS_WARN_THROTTLE(1.0,
            "drone_%d finish exploration. All assigned viewpoints for coverage num = %ld",
            getSlfDroneId(), hetero_manager_->hd_->updated_points_.size());
        break;
      }

      case PLAN_TRAJ: {
        exec_timer_.stop();
        if (fd_->static_state_) {
          fd_->start_pt_ = fd_->odom_pos_;
          fd_->start_vel_ = fd_->odom_vel_;
          fd_->start_acc_.setZero();

          fd_->start_yaw_(0) = fd_->odom_yaw_;
          fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
        }
        else {
          PlannerData* info = &hetero_manager_->hd_->minco_data_;
          double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

          fd_->start_pt_ = info->minco_pos_traj_.getPos(t_r);
          fd_->start_vel_ = info->minco_pos_traj_.getVel(t_r);
          fd_->start_acc_ = info->minco_pos_traj_.getAcc(t_r);
          Eigen::Vector3d ap_yaw = info->minco_yaw_traj_.getPos(t_r);
          fd_->start_yaw_(0) = atan2(ap_yaw(1), ap_yaw(0) + 1e-6);
          fd_->start_yaw_(1) = info->minco_yaw_traj_.getVel(t_r).norm();
          fd_->start_yaw_(2) = info->minco_yaw_traj_.getAcc(t_r).norm();
        }

        // Inform traj_server the replanning
        replan_pub_.publish(std_msgs::Empty());
        int res = callHeterogeneousPlanner();
        if (res == SUCCEED) {
          transitState(PUB_TRAJ, "FSM");
        }
        else if (res == NO_FRONTIER) {
          transitState(FINISH, "FSM");
          fd_->static_state_ = true;
          thread vis_thread(&HeterogeneousPlannerFSM::visualize, this);
          vis_thread.detach();
        }
        else if (res == FAIL) {
          // Still in PLAN_TRAJ state, keep replanning
          ROS_WARN("drone_%d plan fail", getSlfDroneId());
          fd_->static_state_ = true;
        }
        exec_timer_.start();
        break;
      }

      case PUB_TRAJ: {
        PlannerData* info = &hetero_manager_->hd_->minco_data_;
        double dt = (ros::Time::now() - info->start_time_).toSec();
        if (dt > 0) {
          quadrotor_msgs::PolyTraj msg;
          polyTraj2ROSMsg(msg);
          minco_traj_pub_.publish(msg);
          fd_->static_state_ = false;
          transitState(EXEC_TRAJ, "FSM");

          thread vis_thread(&HeterogeneousPlannerFSM::visualize, this);
          vis_thread.detach();
        }
        break;
      }

      case EXEC_TRAJ: {
        PlannerData* info = &hetero_manager_->hd_->minco_data_;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();

        // Replan if traj is almost fully executed
        double time_to_end = info->duration_ - t_cur;

        if (time_to_end < fp_->replan_thresh1_) {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("drone_%d replan: trajectory fully executed================", getSlfDroneId());
          return;
        }
        // Replan if next frontier to be visited is covered
        if (t_cur > fp_->replan_thresh2_ &&
            hetero_manager_->frontier_finder_->isFrontierCovered()) {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("drone_%d replan: frontier cluster covered================", getSlfDroneId());
          return;
        }
        break;
      }
    }
  }
  else {
    switch (state_) {

      case INIT: {
        if (!fd_->have_odom_) {
          ROS_WARN_THROTTLE(1.0, "drone_%d wait for odometry ready.", getSlfDroneId());
          return;
        }
        // Go to wait trigger when odom is ok
        transitState(WAIT_TRIGGER, "FSM");
        break;
      }

      case WAIT_TRIGGER: {
        // Do nothing but wait for trigger
        ROS_INFO_THROTTLE(1.0, "drone_%d wait for trigger.", getSlfDroneId());
        break;
      }

      case FINISH: {
        ROS_WARN_THROTTLE(1.0, "drone_%d finish image acquisition.", getSlfDroneId());
        break;
      }

      case PLAN_TRAJ: {
        if (fd_->static_state_) {
          fd_->start_pt_ = fd_->odom_pos_;
          fd_->start_vel_ = fd_->odom_vel_;
          fd_->start_acc_.setZero();

          fd_->start_yaw_(0) = fd_->odom_yaw_;
          fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
        }
        else {
          // Replan from non-static state, starting from 'replan_time' seconds later
          PlannerData* info = &hetero_manager_->hd_->minco_data_;
          double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

          fd_->start_pt_ = info->minco_pos_traj_.getPos(t_r);
          fd_->start_vel_ = info->minco_pos_traj_.getVel(t_r);
          fd_->start_acc_ = info->minco_pos_traj_.getAcc(t_r);
          fd_->start_yaw_(0) = fd_->odom_yaw_;
          fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
          Eigen::Vector3d ap_pitch = info->minco_pitch_traj_.getPos(t_r);
          fd_->camera_pitch = atan2(ap_pitch(1), ap_pitch(0) + 1e-6);
          Eigen::Vector3d ap_yaw = info->minco_yaw_traj_.getPos(t_r);
          fd_->camera_yaw = atan2(ap_yaw(1), ap_yaw(0) + 1e-6);

          // Pre-filter viewpoints to be visited
          const auto& hd = hetero_manager_->hd_;
          const auto& drone_task = hd->swarm_task_[getSlfDroneId()];
          Eigen::VectorXd durs = info->minco_pos_traj_.getDurations();
          if (durs.size() > 0) {
            double cur_time = 0.0;
            double traj_excu_time = t_r;
            for (int j = 0; j < (int)durs.size(); j++) {
              cur_time += durs(j);
              if (cur_time > traj_excu_time + fp_->replan_thresh3_ * 2) {
                break;
              }
              Eigen::Vector3d traj_pos = info->minco_pos_traj_.getPos(cur_time);
              for (int i = 0; i < (int)drone_task.points_.size(); i++) {
                VectorXd pose(5);
                pose << drone_task.points_[i](0), drone_task.points_[i](1),
                    drone_task.points_[i](2), drone_task.pitchs_[i], drone_task.yaws_[i];
                if (hd->visited_tour_[pose])
                  continue;
                if ((traj_pos - pose.head(3)).norm() < 0.1) {
                  hd->visited_tour_[pose] = true;
                }
              }
            }
          }
        }

        // Inform traj_server the replanning
        replan_pub_.publish(std_msgs::Empty());
        int res = callHeterogeneousPlanner();
        if (res == SUCCEED) {
          transitState(PUB_TRAJ, "FSM");
        }
        else if (res == FAIL) {
          // Still in PLAN_TRAJ state, keep replanning
          ROS_WARN("drone_%d plan fail", getSlfDroneId());
          fd_->static_state_ = true;
        }
        break;
      }

      case PUB_TRAJ: {
        PlannerData* info = &hetero_manager_->hd_->minco_data_;
        double dt = (ros::Time::now() - info->start_time_).toSec();
        if (dt > 0) {
          quadrotor_msgs::PolyTraj msg;
          polyTraj2ROSMsg(msg);
          minco_traj_pub_.publish(msg);
          fd_->static_state_ = false;
          transitState(EXEC_TRAJ, "FSM");

          thread vis_thread(&HeterogeneousPlannerFSM::visualize, this);
          vis_thread.detach();
        }
        break;
      }

      case EXEC_TRAJ: {
        PlannerData* info = &hetero_manager_->hd_->minco_data_;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();

        // Replan if traj is almost fully executed
        double time_to_end = info->duration_ - t_cur;
        if (time_to_end < fp_->replan_thresh1_) {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("drone_%d replan: trajectory fully executed================", getSlfDroneId());
          return;
        }
        // Replan after some time
        if (t_cur > fp_->replan_thresh3_) {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("drone_%d replan: periodic call============================", getSlfDroneId());
        }
        break;
      }
    }
  }
}

int HeterogeneousPlannerFSM::callHeterogeneousPlanner()
{
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
  int res = hetero_manager_->planNextMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
      fd_->start_yaw_, fd_->camera_pitch, fd_->camera_yaw);

  if (res == SUCCEED) {
    auto minco_info = &hetero_manager_->hd_->minco_data_;
    minco_info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
  }
  return res;
}

void HeterogeneousPlannerFSM::polyTraj2ROSMsg(quadrotor_msgs::PolyTraj& poly_msg)
{
  const auto& data = hetero_manager_->hd_->minco_data_;
  const auto& traj = data.minco_pos_traj_;
  const auto& yaw_traj = data.minco_yaw_traj_;
  const auto& pitch_traj = data.minco_pitch_traj_;
  Eigen::VectorXd durs = traj.getDurations();
  Eigen::VectorXd yaw_durs = yaw_traj.getDurations();
  Eigen::VectorXd pitch_durs = pitch_traj.getDurations();
  int piece_num = traj.getPieceNum();
  int yaw_piece_num = yaw_traj.getPieceNum();
  int pitch_piece_num = pitch_traj.getPieceNum();

  poly_msg.drone_id = data.drone_id_;
  poly_msg.traj_id = data.traj_id_;
  poly_msg.order = 7;
  poly_msg.duration.resize(piece_num);
  poly_msg.yaw_duration.resize(yaw_piece_num);
  poly_msg.pitch_duration.resize(yaw_piece_num);
  poly_msg.start_time = data.start_time_;
  poly_msg.coef_x.resize(8 * piece_num);
  poly_msg.coef_y.resize(8 * piece_num);
  poly_msg.coef_z.resize(8 * piece_num);
  poly_msg.yaw_coef_x.resize(8 * yaw_piece_num);
  poly_msg.yaw_coef_y.resize(8 * yaw_piece_num);
  poly_msg.yaw_coef_z.resize(8 * yaw_piece_num);
  poly_msg.pitch_coef_x.resize(8 * pitch_piece_num);
  poly_msg.pitch_coef_y.resize(8 * pitch_piece_num);
  poly_msg.pitch_coef_z.resize(8 * pitch_piece_num);
  const auto& anal = data.minco_pos_anal_;
  const auto& cMat = anal.getCoeffs();
  const auto& yaw_anal = data.minco_yaw_anal_;
  const auto& yaw_cMat = yaw_anal.getCoeffs();
  const auto& pitch_anal = data.minco_pitch_anal_;
  const auto& pitch_cMat = pitch_anal.getCoeffs();
  for (int i = 0; i < piece_num; ++i) {
    poly_msg.duration[i] = durs(i);
    int i8 = i * 8;
    for (int j = 0; j < 8; j++) {
      poly_msg.coef_x[i8 + j] = cMat(j + i8, 0);
      poly_msg.coef_y[i8 + j] = cMat(j + i8, 1);
      poly_msg.coef_z[i8 + j] = cMat(j + i8, 2);
    }
  }
  for (int i = 0; i < yaw_piece_num; ++i) {
    poly_msg.yaw_duration[i] = yaw_durs(i);
    int i8 = i * 8;
    for (int j = 0; j < 8; j++) {
      poly_msg.yaw_coef_x[i8 + j] = yaw_cMat(j + i8, 0);
      poly_msg.yaw_coef_y[i8 + j] = yaw_cMat(j + i8, 1);
      poly_msg.yaw_coef_z[i8 + j] = yaw_cMat(j + i8, 2);
    }
  }
  for (int i = 0; i < pitch_piece_num; ++i) {
    poly_msg.pitch_duration[i] = pitch_durs(i);
    int i8 = i * 8;
    for (int j = 0; j < 8; j++) {
      poly_msg.pitch_coef_x[i8 + j] = pitch_cMat(j + i8, 0);
      poly_msg.pitch_coef_y[i8 + j] = pitch_cMat(j + i8, 1);
      poly_msg.pitch_coef_z[i8 + j] = pitch_cMat(j + i8, 2);
    }
  }
}

void HeterogeneousPlannerFSM::visualize()
{
  auto hd_ptr = hetero_manager_->hd_;
  auto sf = hetero_manager_->frontier_finder_;
  double muti = resolution_ * 10;

  /**********  Draw task allocation  **********/
  if (!isExplorer()) {
    // double r = 1.0;
    // double g = 1 - static_cast<double>(getSlfDroneId()) / (double)hd_ptr->swarm_task_.size();
    // double b = static_cast<double>(getSlfDroneId()) / (double)hd_ptr->swarm_task_.size();

    Eigen::Vector4d col;
    double aa = 0.3;
    int drone_id_ = getSlfDroneId();
    if (drone_id_ == 0)
      col = Eigen::Vector4d(0, 0, 0, 1);
    else if (drone_id_ == 1)
      col = Eigen::Vector4d(1, aa, aa, 1);
    else if (drone_id_ == 2)
      col = Eigen::Vector4d(aa, 1, aa, 1);
    else if (drone_id_ == 3)
      col = Eigen::Vector4d(aa, aa, 1, 1);
    else
      col = Eigen::Vector4d(1, 1, 0, 1);
    visualization_->drawSpheres(hd_ptr->swarm_task_[getSlfDroneId()].points_, 0.6, col,
        "Drone" + to_string(getSlfDroneId()) + "_allocated_points", 0, 7);

    visualization_->drawLines(hd_ptr->final_tour_, 0.08 * muti, col, "final_tour", 0, 7);

    // static int last_allocated_id_num1 = 0;
    // const auto &pos_set = hd_ptr->final_tour_;
    // const auto &text_set = hd_ptr->global_id_;
    // for (size_t i = 0; i < pos_set.size(); i++)
    // {
    //   visualization_->drawText(pos_set[i], to_string(text_set[i]),
    //                            0.4 * muti, Vector4d(1, 1, 1, 1), "global_viewpoint_id", i,
    //                            7);
    // }
    // for (int i = pos_set.size(); i < last_allocated_id_num1; ++i)
    // {
    //   visualization_->drawText(Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0,
    //   0, 1), "global_viewpoint_id", i, 7);
    // }
    // last_allocated_id_num1 = pos_set.size();

    // visualization_->drawSpheres(hd_ptr->path_next_goal_, 0.35, Vector4d(0.0, g, 1.0, 1.0),
    // "A-star waypts", 0, 2);
  }
  /**********  Draw surface viewpoints and updated viewpoints  **********/
  else {
    /**********  Draw frontier  **********/
    // static int last_ftr_num = 0;
    // for (size_t i = 0; i < hd_ptr->frontiers_.size(); ++i)
    // {
    //   visualization_->drawCubes(hd_ptr->frontiers_[i], 0.1,
    //                             visualization_->getColor(double(i) /
    //                             hd_ptr->frontiers_.size(), 0.6), "frontier", i, 4);
    //   // visualization_->drawCubes(hd_ptr->frontiers_[i], 0.1,
    //   //                           Vector4d(1, 0, 0, 0.6), "frontier", i, 4);
    // }
    // for (int i = hd_ptr->frontiers_.size(); i < last_ftr_num; ++i)
    // {
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // }
    // last_ftr_num = hd_ptr->frontiers_.size();

    /**********  Draw surface frontier  **********/
    static int last_sf_ftr_num = 0;
    for (int i = 0; i < (int)hd_ptr->surface_frontiers_.size(); ++i) {
      visualization_->drawCubes(hd_ptr->surface_frontiers_[i], resolution_ * 1.3,
          visualization_->getColor(double(i) / hd_ptr->surface_frontiers_.size(), 1.0),
          "surface_frontier", i, 4);
      // visualization_->drawCubes(hd_ptr->surface_frontiers_[i], resolution_ * 1.3,
      //                           Vector4d(1.0, 0.0, 0.0, 1.0), "surface_frontier", i, 4);
    }
    for (int i = hd_ptr->surface_frontiers_.size(); i < last_sf_ftr_num; ++i) {
      visualization_->drawCubes({}, resolution_, Vector4d(0, 0, 0, 1), "surface_frontier", i, 4);
    }
    last_sf_ftr_num = hd_ptr->surface_frontiers_.size();

    /**********  Draw surface  **********/
    // static int last_surface_num = 0;
    // for (int i = 0; i < (int)hd_ptr->surfaces_.size(); ++i)
    // {
    //   if (hd_ptr->surface_states_[i])
    //     visualization_->drawCubes(hd_ptr->surfaces_[i], resolution_,
    //                               visualization_->getColor(double(i) /
    //                               hd_ptr->surfaces_.size(), 0.2), "surface", i, 4);
    //   else
    //     visualization_->drawCubes(hd_ptr->surfaces_[i], resolution_,
    //                               visualization_->getColor(double(i) /
    //                               hd_ptr->surfaces_.size(), 1.0), "surface", i, 4);
    // }
    // for (int i = hd_ptr->surfaces_.size(); i < last_surface_num; ++i)
    // {
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "surface", i, 4);
    // }
    // last_surface_num = hd_ptr->surfaces_.size();

    /**********  Draw dead surface  **********/
    // static int last_dead_surface_ftr_num = 0;
    // for (size_t i = 0; i < hd_ptr->dead_frontiers_.size(); ++i)
    // {
    //   visualization_->drawCubes(hd_ptr->dead_frontiers_[i], 0.2, Vector4d(1.0, 0.0,
    //   0.0, 1.0), "dead_surface", i, 4);
    // }
    // for (int i = hd_ptr->dead_frontiers_.size(); i < last_dead_surface_ftr_num; ++i)
    // {
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "dead_surface", i, 4);
    // }
    // last_dead_surface_ftr_num = hd_ptr->dead_frontiers_.size();

    /**********  Draw Surface Viewpoints  **********/
    visualization_->drawSpheres(
        hd_ptr->surface_points_, 0.1 * muti, Vector4d(1, 0, 0, 1), "surface_points", 0, 6);
    visualization_->drawLines(hd_ptr->surface_points_, hd_ptr->surface_views_, 0.05 * muti,
        Vector4d(0, 1, 0.5, 1), "surface_view", 0, 6);

    /**********  Draw Updated Surface Viewpoints  **********/
    // visualization_->drawSpheres(hd_ptr->updated_points_, 0.3 * muti, Vector4d(0, 0, 1, 1),
    // "updated_points", 0, 1); visualization_->drawSpheres(hd_ptr->updated_points_, 0.7 * 2.0
    // * 1.0 * tan(35.0 * M_PI / 180.0) *2, Vector4d(0, 0.2, 0.9, 0.3), "updated_bubble", 0, 1);
    // visualization_->drawLines(hd_ptr->updated_points_, hd_ptr->updated_views_, 0.15,
    // Vector4d(0.5, 0.5, 1.0, 1), "updated_views", 0, 1);
    visualization_->drawLines(hd_ptr->updated_views1_, hd_ptr->updated_views2_, 0.035 * muti,
        Vector4d(1.0, 0, 0, 1.0), "updated_view", 0, 1);

    /**********  Draw Text  **********/
    // static int last_text_num1 = 0;
    // for (size_t i = 0; i < hd_ptr->updated_points_.size(); i++)
    // {
    //   visualization_->drawText(hd_ptr->updated_points_[i],
    //   to_string(hd_ptr->updated_iternums_[i]) + "+" + to_string(hd_ptr->updated_counts_[i]),
    //   0.3 * muti, Vector4d(1, 1, 1, 1), "text", i, 1);
    // }
    // for (int i = hd_ptr->updated_points_.size(); i < last_text_num1; ++i)
    // {
    //   visualization_->drawText(Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0,
    //   0, 1), "text", i, 1);
    // }
    // last_text_num1 = hd_ptr->updated_points_.size();

    // visualization_->drawSpheres(hd_ptr->cluster_viewpoints_, 0.3 * muti, Vector4d(0, 0, 1,
    // 1), "cluster_vp", 0, 1);

    // static int last_text_num2 = 0;
    // for (size_t i = 0; i < hd_ptr->cluster_viewpoints_.size(); i++)
    // {
    //   visualization_->drawText(hd_ptr->cluster_viewpoints_[i],
    //   to_string(hd_ptr->cluster_ids_[i]), 0.3 * muti, Vector4d(1, 1, 1, 1), "cluster_text",
    //   i, 2);
    // }
    // for (int i = hd_ptr->cluster_viewpoints_.size(); i < last_text_num2; ++i)
    // {
    //   visualization_->drawText(Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0,
    //   0, 1), "cluster_text", i, 2);
    // }
    // last_text_num2 = hd_ptr->cluster_viewpoints_.size();

    /**********  Draw Cluster Tour  **********/
    for (int i = 0; i < (int)hd_ptr->swarm_task_.size() - 1; i++) {
      if (hd_ptr->drone_cluster_averages_[i].size() == 0)
        continue;
      visualization_->drawSpheres(hd_ptr->drone_cluster_averages_[i], 0.6 * muti,
          visualization_->getColor(double(i) / (hd_ptr->swarm_task_.size() - 1), 0.6),
          "drone" + to_string(i + 1) + "_cluster_average", i, 2);
      visualization_->drawLines(hd_ptr->drone_cluster_averages_[i], 0.1 * muti,
          visualization_->getColor(double(i) / (hd_ptr->swarm_task_.size() - 1), 1.0),
          "drone" + to_string(i + 1) + "_cluster_average_tour", i, 2);
    }

    // visualization_->drawSpheres(hd_ptr->boundary_clusters_, 0.5 * muti, Vector4d(0, 1, 0,
    // 0.6), "boundary_cluster_average", 1, 1);
    // visualization_->drawSpheres(hd_ptr->all_boundary_clusters_, 0.4 * muti, Vector4d(0, 0, 1,
    // 0.6), "all_boundary_cluster_average", 1, 1);

    /**********  Draw viewpoint cluster text **********/
    // visualization_->drawSpheres(hd_ptr->cluster_averages_, 0.6 * muti, Vector4d(1, 0, 0, 0.8),
    // "cluster_average", 1, 1);

    // static int last_text_num3 = 0;
    // for (size_t i = 0; i < hd_ptr->cluster_averages_.size(); i++) {
    //   visualization_->drawText(hd_ptr->cluster_averages_[i], to_string(i), 0.8 * muti,
    //       Vector4d(1, 1, 1, 1), "cluster_average_text", i, 2);
    // }
    // for (int i = hd_ptr->cluster_viewpoints_.size(); i < last_text_num3; ++i) {
    //   visualization_->drawText(Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0, 0, 1),
    //       "cluster_average_text", i, 2);
    // }
    // last_text_num3 = hd_ptr->cluster_averages_.size();

    /**********  Draw Viewpoint Allocation  **********/
    // for (int i = 0; i < (int)hd_ptr->swarm_task_.size(); i++)
    // {
    //   visualization_->drawSpheres(hd_ptr->swarm_task_[i].points_, 0.3 * muti,
    //                               visualization_->getColor(double(i) /
    //                               hd_ptr->swarm_task_.size(), 1.0),
    //                               "all_assigned_viewpoints", i, 7);
    //   visualization_->drawLines(hd_ptr->swarm_task_[i].points_, 0.06 * muti,
    //                             visualization_->getColor(double(i) /
    //                             hd_ptr->swarm_task_.size(), 1.0), "all_final_tour", i, 7);
    // }
  }
}

void HeterogeneousPlannerFSM::frontierCallback(const ros::TimerEvent& e)
{
  if (getSlfDroneId() != 0)
    return;

  if (state_ == WAIT_TRIGGER) {
    auto sf = hetero_manager_->frontier_finder_;
    auto hd = hetero_manager_->hd_;
    sf->searchFrontiers();
    sf->computeFrontiersToVisit();
    sf->updateFrontierCostMatrix();
    sf->getFrontiers(hd->surface_frontiers_);

    // Draw frontier
    auto hd_ptr = hetero_manager_->hd_;
    static int last_sf_ftr_num = 0;
    for (int i = 0; i < (int)hd_ptr->surface_frontiers_.size(); ++i) {
      visualization_->drawCubes(hd_ptr->surface_frontiers_[i], resolution_ * 1.3,
          visualization_->getColor(double(i) / hd_ptr->surface_frontiers_.size(), 1.0),
          "surface_frontier", i, 4);
      // visualization_->drawCubes(hd_ptr->surface_frontiers_[i], resolution_ * 1.3,
      //                           Vector4d(1.0, 0.0, 0.0, 1.0), "surface_frontier", i, 4);
    }
    for (int i = hd_ptr->surface_frontiers_.size(); i < last_sf_ftr_num; ++i) {
      visualization_->drawCubes({}, resolution_, Vector4d(0, 0, 0, 1), "surface_frontier", i, 4);
    }
    last_sf_ftr_num = hd_ptr->surface_frontiers_.size();
  }
}

void HeterogeneousPlannerFSM::triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (state_ != WAIT_TRIGGER)
    return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}

void HeterogeneousPlannerFSM::safetyCallback(const ros::TimerEvent& e)
{
  if (isExplorer()) {
    if (state_ == FSM_STATE::EXEC_TRAJ) {
      // Check safety and trigger replan if necessary
      bool safe = hetero_manager_->checkTrajCollision();
      if (!safe) {
        fd_->static_state_ = true;
        ROS_WARN("Replan: collision detected==================================");
        transitState(PLAN_TRAJ, "safetyCallback");
      }
    }
  }
  else {
    if (state_ == FSM_STATE::EXEC_TRAJ) {
      // Check safety and trigger replan if necessary
      bool safe = hetero_manager_->checkTrajCollision();
      if (!safe) {
        fd_->static_state_ = true;
        ROS_WARN("Replan: collision detected==================================");
        transitState(PLAN_TRAJ, "safetyCallback");
      }

      PlannerData* info = &hetero_manager_->hd_->minco_data_;
      double t_r = (ros::Time::now() - info->start_time_).toSec();
      Eigen::Vector3d ap_pitch = info->minco_pitch_traj_.getPos(t_r);
      fd_->odom_camera_pitch_ = atan2(ap_pitch(1), ap_pitch(0) + 1e-6);
      Eigen::Vector3d ap_yaw = info->minco_yaw_traj_.getPos(t_r);
      fd_->odom_camera_yaw_ = atan2(ap_yaw(1), ap_yaw(0) + 1e-6);
    }
    hetero_manager_->surface_coverage_->updateSeenCells(
        fd_->odom_pos_, fd_->odom_camera_pitch_, fd_->odom_camera_yaw_);
  }
}

void HeterogeneousPlannerFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;

  // Only photographers need update visited tour
  if (isExplorer())
    return;

  const auto& hd = hetero_manager_->hd_;
  const auto& drone_task = hd->swarm_task_[getSlfDroneId()];
  // Update visited tour
  for (int i = 0; i < (int)drone_task.points_.size(); i++) {
    VectorXd pose(5);
    pose << drone_task.points_[i](0), drone_task.points_[i](1), drone_task.points_[i](2),
        drone_task.pitchs_[i], drone_task.yaws_[i];
    if (hd->visited_tour_[pose])
      continue;
    if ((fd_->odom_pos_ - pose.head(3)).norm() < 0.2) {
      hd->visited_tour_[pose] = true;
    }
  }
}

void HeterogeneousPlannerFSM::transitState(FSM_STATE new_state, string pos_call)
{
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " +
              fd_->state_str_[int(new_state)]
       << endl;
}

void HeterogeneousPlannerFSM::taskAssignmentTimerCallback(const ros::TimerEvent& e)
{
  // Explorer send task assignment information
  if (state_ == INIT || state_ == WAIT_TRIGGER)
    return;

  // Only explorer does task assignment
  if (!isExplorer())
    return;

  const auto& hd = hetero_manager_->hd_;
  for (size_t i = 0; i < hd->swarm_task_.size(); i++) {
    if (hd->swarm_task_[i].points_.size() == 0)
      continue;

    heterogeneous_manager::ViewpointsTask vp_task;
    vp_task.drone_id = hd->swarm_task_[i].drone_id_;
    vp_task.stamp = hd->swarm_task_[i].stamp_;
    for (const auto& point : hd->swarm_task_[i].points_) {
      geometry_msgs::Point ros_point;
      ros_point.x = point.x();
      ros_point.y = point.y();
      ros_point.z = point.z();
      vp_task.points.push_back(ros_point);
    }
    vp_task.orders = hd->swarm_task_[i].orders_;
    vp_task.pitchs = hd->swarm_task_[i].pitchs_;
    vp_task.yaws = hd->swarm_task_[i].yaws_;
    task_assignment_pub_.publish(vp_task);
  }
}

void HeterogeneousPlannerFSM::taskAssignmentMsgCallback(
    const heterogeneous_manager::ViewpointsTaskConstPtr& msg)
{
  // Photographers get task assignment information
  if (isExplorer())
    return;

  if (msg->drone_id >= drone_num_)
    return;

  // Every photographer has all photographers' task asignment information
  // but only excute its own tasks
  auto& hd = hetero_manager_->hd_;
  hd->swarm_task_[msg->drone_id].drone_id_ = msg->drone_id;
  hd->swarm_task_[msg->drone_id].stamp_ = msg->stamp;
  hd->swarm_task_[msg->drone_id].points_.clear();
  for (const auto& ros_point : msg->points) {
    Eigen::Vector3d point;
    point.x() = ros_point.x;
    point.y() = ros_point.y;
    point.z() = ros_point.z;
    hd->swarm_task_[msg->drone_id].points_.push_back(point);
  }
  hd->swarm_task_[msg->drone_id].orders_ = msg->orders;
  hd->swarm_task_[msg->drone_id].pitchs_ = msg->pitchs;
  hd->swarm_task_[msg->drone_id].yaws_ = msg->yaws;
}

void HeterogeneousPlannerFSM::droneStateTimerCallback(const ros::TimerEvent& e)
{
  if (state_ == INIT)
    return;

  // Broadcast own state periodically
  heterogeneous_manager::DroneState msg;
  msg.drone_id = getSlfDroneId();

  auto& state = hetero_manager_->hd_->swarm_state_[msg.drone_id];

  if (fd_->static_state_) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  }
  else {
    if (isExplorer()) {
      const auto& info = hetero_manager_->hd_->minco_data_;
      double t_r = (ros::Time::now() - info.start_time_).toSec();
      state.pos_ = info.minco_pos_traj_.getPos(t_r);
      state.vel_ = info.minco_pos_traj_.getVel(t_r);
      Eigen::Vector3d ap = info.minco_yaw_traj_.getPos(t_r);
      double yaw = atan2(ap(1), ap(0) + 1e-6);
      state.yaw_ = yaw;
    }
    else {
      // Photographers' minco trajectories' pitch and yaw are gimbal camera's pitch and yaw
      // so can't like the explorer use minco's yaw
      const auto& info = hetero_manager_->hd_->minco_data_;
      double t_r = (ros::Time::now() - info.start_time_).toSec();
      state.pos_ = info.minco_pos_traj_.getPos(t_r);
      state.vel_ = info.minco_pos_traj_.getVel(t_r);
      state.yaw_ = fd_->odom_yaw_;
    }
  }
  state.stamp_ = ros::Time::now().toSec();
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.pitch = 0.0;
  msg.yaw = state.yaw_;

  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;

  drone_state_pub_.publish(msg);
}

void HeterogeneousPlannerFSM::droneStateMsgCallback(
    const heterogeneous_manager::DroneStateConstPtr& msg)
{
  // Only update other drones' states
  if (msg->drone_id == getSlfDroneId())
    return;

  auto& drone_state = hetero_manager_->hd_->swarm_state_[msg->drone_id];
  // Avoid unordered msg
  if (drone_state.stamp_ + 1e-4 >= msg->stamp)
    return;

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.pitch_ = msg->pitch;
  drone_state.yaw_ = msg->yaw;
  drone_state.stamp_ = msg->stamp;
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;
}

void HeterogeneousPlannerFSM::visitedTourTimerCallback(const ros::TimerEvent& e)
{
  // Photographers send visited viewpoints each other
  if (state_ == INIT || state_ == WAIT_TRIGGER)
    return;

  // Only photographers need update visited tour
  if (isExplorer())
    return;

  const auto& hd = hetero_manager_->hd_;
  heterogeneous_manager::VisitedTour VT;
  VT.drone_id = getSlfDroneId();
  VT.stamp = ros::Time::now().toSec();
  for (const auto& pair : hd->visited_tour_) {
    // if the viewpoint visited
    if (pair.second) {
      // x,y,z,pitch,yaw
      geometry_msgs::Point ros_point;
      ros_point.x = pair.first(0);
      ros_point.y = pair.first(1);
      ros_point.z = pair.first(2);
      VT.points.push_back(ros_point);
      VT.pitchs.push_back(pair.first(3));
      VT.yaws.push_back(pair.first(4));
    }
  }
  visited_tour_pub_.publish(VT);
}

void HeterogeneousPlannerFSM::visitedTourMsgCallback(
    const heterogeneous_manager::VisitedTourConstPtr& msg)
{
  // Only photographers need visited tour information
  if (!isExplorer())
    return;
  if (msg->drone_id >= drone_num_)
    return;

  const auto& hd = hetero_manager_->hd_;

  for (int i = 0; i < (int)msg->points.size(); i++) {
    VectorXd pose(5);
    pose << msg->points[i].x, msg->points[i].y, msg->points[i].z, msg->pitchs[i], msg->yaws[i];
    // record the viewpoint is visited
    hd->visited_tour_[pose] = true;
  }
}

int HeterogeneousPlannerFSM::getSlfDroneId()
{
  return hetero_manager_->hp_->drone_id_;
}

int HeterogeneousPlannerFSM::isExplorer()
{
  return hetero_manager_->frontier_finder_->isExplorer();
}
}  // namespace hetero_planner
