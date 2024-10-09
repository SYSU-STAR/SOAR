// #include <fstream>
#include <heterogeneous_manager/heterogeneous_planner_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <random>
#include <numeric>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>

#include <heterogeneous_manager/hetero_data.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace hetero_planner {
HeterogenousPlannerManager::HeterogenousPlannerManager()
{
}

HeterogenousPlannerManager::~HeterogenousPlannerManager()
{
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void HeterogenousPlannerManager::initialize(ros::NodeHandle& nh)
{
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  path_finder_.reset(new Astar);
  path_finder_->init(nh, edt_environment_);
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  surface_coverage_.reset(new SurfaceCoverage(edt_environment_, nh));

  traj_generator_.reset(new TrajGenerator);
  traj_generator_->init(edt_environment_, nh);

  hd_.reset(new HeteroData);
  hp_.reset(new HeteroParam);

  nh.param("exploration/mtsp_dir", hp_->mtsp_dir_, string("null"));
  nh.param("exploration/drone_id", hp_->drone_id_, -1);
  nh.param("exploration/drone_num", hp_->drone_num_, -1);

  hd_->swarm_state_.resize(hp_->drone_num_);
  hd_->swarm_task_.resize(hp_->drone_num_);
  hd_->drone_cluster_averages_.resize(hp_->drone_num_ - 1);
  for (int i = 0; i < hp_->drone_num_; ++i) {
    hd_->swarm_state_[i].stamp_ = 0.0;
    hd_->swarm_task_[i].stamp_ = 0.0;
  }

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);

  nh.param("viewpoint_cluster/max_cluster_dist", max_cluster_dist_, -1.0);
  nh.param("viewpoint_cluster/viewpoint_h_cost", viewpoint_h_cost_, 0.5);

  nh.param("surface/surface_min_update_time", surface_min_update_time_, -1.0);

  // TODO: Convert to parameters.
  max_local_num_ = 20;
  consistent_alpha_ = 0.25;
  consistent_value_ = 50;
  mdmtsp_iterations_ = 700;

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution, origin);

  tsp_client_ =
      nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(hp_->drone_id_), true);

  VTR.init_tour_ = false;
  VTR.vp_num_ = 0;

  VAR.init_assignment_ = false;
  VAR.cluster_num_ = 0;
  VAR.last_cluster_num_ = 0;
  VAR.drone_assigned_cluster_ids_.resize(hp_->drone_num_ - 1);  // drone 1~N are photographers
  VAR.cluster_h_cost_ = max_cluster_dist_ / 2 * viewpoint_h_cost_ + 0.1;

  final_task_assignment_ = false;
}

int HeterogenousPlannerManager::planNextMotion(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const double& camera_pitch, const double& camera_yaw)
{
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  hd_->views_.clear();
  hd_->surface_views_.clear();
  hd_->global_tour_.clear();

  Vector3d next_pos;
  double next_yaw;
  if (frontier_finder_->isExplorer()) {
    /******** surface frontier-based exploration ********/

    frontier_finder_->searchFrontiers();
    frontier_finder_->computeFrontiersToVisit();

    frontier_finder_->getFrontiers(hd_->frontiers_);
    frontier_finder_->getFrontierBoxes(hd_->frontier_boxes_);
    frontier_finder_->getDormantFrontiers(hd_->dead_frontiers_);

    frontier_finder_->getTopViewpointsInfo(pos, hd_->points_, hd_->yaws_, hd_->averages_);
    for (size_t i = 0; i < hd_->points_.size(); ++i)
      hd_->views_.push_back(
          hd_->points_[i] + 1.0 * Vector3d(cos(hd_->yaws_[i]), sin(hd_->yaws_[i]), 0));

    double frontier_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    /******** Incremental Viewpoint Generation for Coverage ********/

    // If no surface frontier exists, exploration is finished, and this is the final surface
    // calculation and task allocation.
    if (hd_->frontiers_.empty()) {
      surface_coverage_->setVpGenStartFlag(true);
      final_task_assignment_ = true;
    }

    surface_coverage_->setSurfaceFrontiers(hd_->frontiers_);
    surface_coverage_->searchSurfaces();
    surface_coverage_->computeSurfacesToVisit();
    if ((ros::Time::now() - hd_->surface_update_time_).toSec() > surface_min_update_time_) {
      surface_coverage_->updateSurfaceState();
      hd_->surface_update_time_ = ros::Time::now();
    }
    if (hd_->frontiers_.empty()) {
      surface_coverage_->updateSurfaceState();
    }
    surface_coverage_->surfaceViewpointsGeneration();
    double surface_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    surface_coverage_->getSurfaces(hd_->surfaces_);
    surface_coverage_->getDormantSurfaces(hd_->dead_surfaces_);
    surface_coverage_->getSurfacesState(hd_->surface_states_);
    surface_coverage_->getSurfaceFrontiers(hd_->surface_frontiers_);

    /* For Final Viewpoints Visualization */
    vector<VectorXd> final_vps;
    vector<int> final_vp_ids;
    surface_coverage_->getFinalViewpoints(
        final_vps, final_vp_ids, hd_->updated_iternums_, hd_->updated_counts_);
    unordered_map<VectorXd, int> final_vp_id_finder;
    for (int i = 0; i < (int)final_vps.size(); i++)
      final_vp_id_finder[final_vps[i]] = final_vp_ids[i];

    vector<double> updated_yaws, updated_pitchs;
    hd_->updated_views_.clear();
    hd_->updated_points_.clear();
    hd_->updated_views1_.clear();
    hd_->updated_views2_.clear();
    for (auto vp : final_vps) {
      hd_->updated_points_.push_back(Vector3d(vp(0), vp(1), vp(2)));
      updated_pitchs.push_back(vp(3));
      updated_yaws.push_back(vp(4));
    }
    for (size_t i = 0; i < hd_->updated_points_.size(); ++i) {
      Vector3d view =
          hd_->updated_points_[i] + 1.0 * Vector3d(cos(updated_yaws[i]), sin(updated_yaws[i]), 0);
      hd_->updated_views_.push_back(view);

      vector<Vector3d> v1, v2;
      surface_coverage_->percep_utils_->setPose_PY(
          hd_->updated_points_[i], updated_pitchs[i], updated_yaws[i]);
      surface_coverage_->percep_utils_->getFOV_PY(v1, v2);
      hd_->updated_views1_.insert(hd_->updated_views1_.end(), v1.begin(), v1.end());
      hd_->updated_views2_.insert(hd_->updated_views2_.end(), v2.begin(), v2.end());
    }

    surface_coverage_->getAllSurfaceViewpointsInfo(
        hd_->surface_points_, hd_->surface_yaws_, hd_->surface_pitchs_);
    for (size_t i = 0; i < hd_->surface_points_.size(); ++i) {
      Vector3d point = hd_->surface_points_[i];
      double yaw = hd_->surface_yaws_[i];
      double pitch = hd_->surface_pitchs_[i];
      double radius = 2.0;
      Vector3d new_position;
      new_position.x() = point.x() + radius * cos(pitch) * cos(yaw);
      new_position.y() = point.y() + radius * cos(pitch) * sin(yaw);
      new_position.z() = point.z() + radius * sin(pitch);
      hd_->surface_views_.push_back(new_position);
    }
    /* For Final Viewpoints Visualization End */

    // calculate next best viewpoint for exploration
    if (hd_->points_.size() > 1) {
      vector<int> indices;
      findGlobalTour(pos, vel, yaw, indices);

      next_pos = hd_->points_[indices[0]];
      next_yaw = hd_->yaws_[indices[0]];
    }
    else if (hd_->points_.size() == 1) {
      hd_->global_tour_ = { pos, hd_->points_[0] };

      next_pos = hd_->points_[0];
      next_yaw = hd_->yaws_[0];
    }
    else
      ROS_ERROR("%sEmpty destination.", color_map_[hp_->drone_id_].c_str());

    double exploration_tsp_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    ROS_WARN("%sdrone_%d frontier: size = %ld, time = %lf || surface: size = %ld, time = %lf",
        color_map_[hp_->drone_id_].c_str(), hp_->drone_id_, hd_->frontiers_.size(), frontier_time,
        hd_->surface_points_.size(), surface_time);

    ROS_WARN("%sdrone_%d exploration tsp time = %lf", color_map_[hp_->drone_id_].c_str(),
        hp_->drone_id_, exploration_tsp_time);

    /****************************** Task Assignment ******************************/
    if ((int)final_vps.size() >= 1 && hp_->drone_num_ > 1) {
      /****** viewpoint cluster ******/
      bool vp_cluster_changed = viewpointsClustering(final_vps);

      if (vp_cluster_changed) {
        // Update visualization
        hd_->cluster_viewpoints_.clear();
        hd_->cluster_ids_.clear();
        hd_->cluster_averages_.clear();
        hd_->cluster_num_ = VAR.cluster_num_;
        for (auto vp : final_vps) {
          hd_->cluster_viewpoints_.push_back(vp.head(3));
          hd_->cluster_ids_.push_back(VAR.vp_cluster_finder_[vp]);
        }
        for (int i = 0; i < (int)VAR.vp_clusters_.size(); i++) {
          hd_->cluster_averages_.push_back(VAR.vp_clusters_[i].average_);
        }

        // Find newly visited viewpoints and dynamically update viewpoint clustering information.
        vector<VectorXd> all_visited_vps;
        vector<VectorXd> new_visited_vps;
        for (const auto& pair : hd_->visited_tour_)
          if (pair.second)
            all_visited_vps.push_back(pair.first);

        if (hd_->last_visited_vps_.size() != all_visited_vps.size()) {
          for (auto visited_vp : all_visited_vps) {
            bool is_last = false;
            for (auto last_vp : hd_->last_visited_vps_) {
              if ((visited_vp - last_vp).norm() < 1e-3) {
                is_last = true;
                break;
              }
            }
            if (!is_last)
              new_visited_vps.push_back(visited_vp);
          }
          ROS_WARN("[Visited Viewpoints] total num: now = %ld last = %ld new  = %ld",
              all_visited_vps.size(), hd_->last_visited_vps_.size(), new_visited_vps.size());

          // Dynamically update viewpoint information.
          for (auto new_vp : new_visited_vps) {
            int cluster_id = VAR.vp_cluster_finder_[new_vp];
            double before_cluster_cost = VAR.vp_clusters_[cluster_id].cost_;
            VAR.vp_clusters_[cluster_id].cost_ -= VAR.cluster_h_cost_;
            ROS_WARN("[Visited Viewpoints] delete vp: cls_id = %d before_cost = %f after_cost = %f",
                cluster_id, before_cluster_cost, VAR.vp_clusters_[cluster_id].cost_);
          }
          hd_->last_visited_vps_ = all_visited_vps;
        }

        // Reallocation occurs only when new clusters are generated.
        if (VAR.last_cluster_num_ < (int)VAR.vp_clusters_.size() || final_task_assignment_) {
          // photographer id from 1 to drone_num_
          vector<VectorXd> photographers_pose;
          for (int i = 1; i < (int)hd_->swarm_state_.size(); i++) {
            VectorXd pose(5);
            const auto& state = hd_->swarm_state_[i];
            pose << state.pos_(0), state.pos_(1), state.pos_(2), state.pitch_, state.yaw_;
            photographers_pose.push_back(pose);
          }
          int cluster_num = (int)VAR.vp_clusters_.size();

          // Maintain the A* matrix for newly added viewpoint clusters.
          t1 = ros::Time::now();
          updateViewpointClustersMatrix();
          ROS_WARN("[Task Assignment] viewpoint clusters cost matrix: time = %f",
              (ros::Time::now() - t1).toSec());

          if (!VAR.init_assignment_) {
            // Initialize task allocation using MTSP.
            if (cluster_num >= hp_->drone_num_ - 1) {
              findClustersDronesTour(
                  VAR.vp_clusters_, photographers_pose, VAR.drone_assigned_cluster_ids_);
              for (int i = 0; i < (int)VAR.drone_assigned_cluster_ids_.size(); i++)
                for (auto cluster_id : VAR.drone_assigned_cluster_ids_[i])
                  VAR.cluster_drone_finder_[cluster_id] = i + 1;  // drone_id = i+1

              if (cluster_num >= (hp_->drone_num_ - 1) * 2) {
                ROS_WARN("[Task Assignment] MTSP INIT FINSIH");
                VAR.init_assignment_ = true;
              }
            }
            else {
              // Too few clusters; allocate based on distance.
              VAR.drone_assigned_cluster_ids_.clear();
              VAR.drone_assigned_cluster_ids_.resize(photographers_pose.size());
              ROS_WARN("[Task Assignment] cluster num = %d is too little", cluster_num);
              vector<char> have_task;
              have_task.resize(photographers_pose.size(), 0);
              for (int i = 0; i < (int)VAR.vp_clusters_.size(); i++) {
                int min_dist = 1000000, min_drone_id = 0;
                for (int j = 0; j < (int)photographers_pose.size(); j++) {
                  if (have_task[j] == 1)
                    continue;
                  double dist =
                      (photographers_pose[j].head(3) - VAR.vp_clusters_[i].average_).norm();
                  if (dist < min_dist) {
                    min_dist = dist;
                    min_drone_id = j + 1;  // drone_id = j+1
                  }
                }
                have_task[min_drone_id - 1] = 1;
                VAR.drone_assigned_cluster_ids_[min_drone_id - 1].push_back(
                    VAR.vp_clusters_[i].cluster_id_);
                VAR.cluster_drone_finder_[VAR.vp_clusters_[i].cluster_id_] = min_drone_id;
              }
            }
          }
          else {
            /* Consistent-MDMTSP */

            // Obtain currently unvisited clusters and store their IDs in existing_cluster_indices.
            vector<int> existing_cluster_indices;
            for (int i = 0; i < cluster_num; i++)
              if (VAR.vp_clusters_[i].cost_ > -1e-3)  // Skip clusters without viewpoints.
                existing_cluster_indices.push_back(i);
            ROS_WARN(
                "[Task Assignment] existing cluster size: %ld", existing_cluster_indices.size());

            auto last_drone_assigned_cluster_ids = VAR.drone_assigned_cluster_ids_;

            vector<vector<int>> last_drone_existing_cluster_ids;
            last_drone_existing_cluster_ids.resize(last_drone_assigned_cluster_ids.size());

            vector<char> tmp_flags;
            tmp_flags.resize(existing_cluster_indices.size(), 0);

            for (int i = 0; i < (int)last_drone_assigned_cluster_ids.size(); i++) {
              auto cls_ids = last_drone_assigned_cluster_ids[i];
              for (auto cls_id : cls_ids) {
                int existing_id = -1;
                for (int j = 0; j < (int)existing_cluster_indices.size(); j++) {
                  if (!tmp_flags[j] && existing_cluster_indices[j] == cls_id) {
                    tmp_flags[j] = 1;
                    existing_id = j;
                    break;
                  }
                }
                if (existing_id != -1)
                  last_drone_existing_cluster_ids[i].push_back(existing_id);
              }
            }

            vector<int> new_cluster_ids, last_cluster_ids;
            for (int i = 0; i < (int)last_drone_existing_cluster_ids.size(); i++) {
              auto ids = last_drone_existing_cluster_ids[i];
              for (auto cluster_id : ids) last_cluster_ids.push_back(cluster_id);
            }

            for (int i = 0; i < (int)tmp_flags.size(); i++)
              if (!tmp_flags[i])
                new_cluster_ids.push_back(i);

            // Final viewpoint clustering task allocation results
            // (corresponding to all cluster IDs).
            vector<vector<int>> final_assigned_cluster_indices;
            bool res;

            if (!final_task_assignment_)
              res = taskAssignment(photographers_pose, existing_cluster_indices,
                  last_drone_existing_cluster_ids, final_assigned_cluster_indices, last_cluster_ids,
                  new_cluster_ids);
            else {
              res = true;
              findClustersDronesTourV2(
                  photographers_pose, existing_cluster_indices, final_assigned_cluster_indices);
            }

            if (res) {
              vector<vector<int>> final_assigned_cluster_ids;
              final_assigned_cluster_ids.resize(final_assigned_cluster_indices.size());
              // Convert to actual cluster IDs.
              for (int i = 0; i < (int)final_assigned_cluster_indices.size(); i++)
                for (auto index : final_assigned_cluster_indices[i])
                  final_assigned_cluster_ids[i].push_back(existing_cluster_indices[index]);
              VAR.drone_assigned_cluster_ids_ = final_assigned_cluster_ids;
            }
          }

          // Update cluster allocation status.
          for (int i = 0; i < (int)VAR.drone_assigned_cluster_ids_.size(); i++)
            for (auto cluster_id : VAR.drone_assigned_cluster_ids_[i])
              VAR.cluster_drone_finder_[cluster_id] = i + 1;

          // Update visualization.
          hd_->drone_cluster_averages_.clear();
          hd_->drone_cluster_averages_.resize(photographers_pose.size());
          for (int i = 0; i < (int)photographers_pose.size(); i++)
            for (auto cluster_id : VAR.drone_assigned_cluster_ids_[i])
              hd_->drone_cluster_averages_[i].push_back(VAR.vp_clusters_[cluster_id].average_);

          // Summarize task allocation.
          for (int i = 0; i < (int)photographers_pose.size(); i++) {
            hd_->swarm_task_[i].orders_.clear();
            hd_->swarm_task_[i].points_.clear();
            hd_->swarm_task_[i].pitchs_.clear();
            hd_->swarm_task_[i].yaws_.clear();

            hd_->swarm_task_[i].drone_id_ = i + 1;  // drone id = i+1
            hd_->swarm_task_[i].stamp_ = ros::Time::now().toSec();
            int tour_order = 0;
            for (auto cluster_id : VAR.drone_assigned_cluster_ids_[i]) {
              for (auto vp : VAR.vp_clusters_[cluster_id].viewpoints_) {
                if (hd_->visited_tour_[vp])
                  continue;
                hd_->swarm_task_[i].orders_.push_back(tour_order);
                hd_->swarm_task_[i].points_.push_back(vp.head(3));
                hd_->swarm_task_[i].pitchs_.push_back(vp(3));
                hd_->swarm_task_[i].yaws_.push_back(vp(4));
              }
              tour_order++;
            }
            ROS_INFO("[Task Assignment] drone_%d assigned viewpoints size = %d", i + 1,
                (int)hd_->swarm_task_[i].points_.size());
          }
          VAR.last_cluster_num_ = VAR.vp_clusters_.size();
        }
      }
    }

    if (hd_->frontiers_.empty()) {
      ROS_ERROR("%sdrone_%d No frontier.", color_map_[hp_->drone_id_].c_str(), hp_->drone_id_);
      return NO_FRONTIER;
    }
  }
  else  // photographers find [Tour of assigned viewpoints]
  {
    const auto& drone_task = hd_->swarm_task_[hp_->drone_id_];

    std::vector<Eigen::VectorXd> task_viewpoints;
    for (int i = 0; i < (int)drone_task.orders_.size(); i++) {
      VectorXd pose(5);
      pose << drone_task.points_[i](0), drone_task.points_[i](1), drone_task.points_[i](2),
          drone_task.pitchs_[i], drone_task.yaws_[i];
      if (hd_->visited_tour_[pose])
        continue;
      task_viewpoints.push_back(pose);
      if ((int)task_viewpoints.size() >= max_local_num_)
        break;
    }
    vector<int> tour_indices;
    if (task_viewpoints.size() == 0)
      return NO_RESULT;
    else if (task_viewpoints.size() > 1)
      findViewpointsTour(pos, vel, camera_yaw, camera_pitch, task_viewpoints, tour_indices);
    else
      tour_indices.push_back(0);

    hd_->final_tour_.clear();
    hd_->global_id_.clear();
    hd_->final_pitch_.clear();
    hd_->final_yaw_.clear();
    for (int i = 0; i < (int)tour_indices.size(); i++) {
      int tour_id = tour_indices[i];
      if (hd_->visited_tour_[task_viewpoints[tour_id]])
        continue;
      Vector3d tour_pos = task_viewpoints[tour_id].head(3);
      double tour_pitch = task_viewpoints[tour_id](3);
      double tour_yaw = task_viewpoints[tour_id](4);
      hd_->final_tour_.push_back(tour_pos);
      hd_->global_id_.push_back(0);
      hd_->final_pitch_.push_back(tour_pitch);
      hd_->final_yaw_.push_back(tour_yaw);
    }

    hd_->last_self_task_ = hd_->swarm_task_[hp_->drone_id_];
  }

  if (frontier_finder_->isExplorer()) {
    t1 = ros::Time::now();
    vector<Eigen::Vector3d> waypts;
    vector<bool> waypts_indexs;
    vector<double> given_pitch, given_yaw;
    double start_pitch, end_pitch;
    double start_yaw, end_yaw;
    hd_->path_next_goal_.clear();
    std::vector<Eigen::Vector3d> tmp_path;
    vector<Eigen::VectorXd> updates_waypts;

    VectorXd start(5), end(5);
    path_finder_->reset();
    path_finder_->setResolution(0.5);
    if (path_finder_->search(pos, next_pos) != Astar::REACH_END) {
      return FAIL;
    }
    tmp_path = path_finder_->getPath();
    shortenPath(tmp_path);

    start_pitch = 0.0;
    start_yaw = yaw(0);
    end_pitch = 0.0;
    end_yaw = next_yaw;
    start << pos(0), pos(1), pos(2), start_pitch, start_yaw;
    end << next_pos(0), next_pos(1), next_pos(2), end_pitch, end_yaw;
    AngleInterpolation(start, end, tmp_path, updates_waypts);
    given_pitch.push_back(start_pitch);
    given_yaw.push_back(start_yaw);

    // avoid repeat path so i start with 1
    for (int j = 1; j < (int)updates_waypts.size(); j++) {
      Vector3d pos;
      pos << updates_waypts[j](0), updates_waypts[j](1), updates_waypts[j](2);
      waypts.push_back(pos);
      hd_->path_next_goal_.push_back(pos);
      given_pitch.push_back(0.0);
      given_yaw.push_back(updates_waypts[j](4));
      if (j == (int)updates_waypts.size() - 1)
        waypts_indexs.push_back(false);
      else
        waypts_indexs.push_back(true);
    }

    if (waypts.size() == 1)
      return NO_RESULT;

    Vector3d odom = pos;
    traj_generator_->HCTraj(odom, vel, acc, waypts, waypts_indexs, given_pitch, given_yaw);
    traj_generator_->getMincoPosTraj(
        hd_->minco_data_.minco_pos_traj_, hd_->minco_data_.minco_pos_anal_);
    traj_generator_->getMincoPitchTraj(
        hd_->minco_data_.minco_pitch_traj_, hd_->minco_data_.minco_pitch_anal_);
    traj_generator_->getMincoYawTraj(
        hd_->minco_data_.minco_yaw_traj_, hd_->minco_data_.minco_yaw_anal_);
    static int traj_id_ = 1;
    hd_->minco_data_.drone_id_ = hp_->drone_id_;
    hd_->minco_data_.start_time_ = ros::Time::now();
    hd_->minco_data_.traj_id_ = traj_id_;
    hd_->minco_data_.duration_ = hd_->minco_data_.minco_pos_traj_.getTotalDuration();
    traj_id_++;
    ROS_WARN("%sdrone_%d planNextMotion time: %lf", color_map_[hp_->drone_id_].c_str(),
        hp_->drone_id_, (ros::Time::now() - t1).toSec());
  }
  else {
    if (hd_->final_tour_.size() == 0)
      return NO_RESULT;
    else {
      vector<Eigen::Vector3d> waypts;
      vector<bool> waypts_indexs;
      vector<double> given_pitch, given_yaw;
      double start_pitch, end_pitch;
      double start_yaw, end_yaw;
      hd_->path_next_goal_.clear();
      std::vector<Eigen::Vector3d> tmp_path;
      vector<Eigen::VectorXd> updates_waypts;

      for (int i = 0; i < min((int)hd_->final_tour_.size(), 3); i++) {
        VectorXd start(5), end(5);
        if (i == 0) {
          path_finder_->reset();
          path_finder_->setResolution(0.5);
          if (path_finder_->search(pos, hd_->final_tour_[0]) != Astar::REACH_END) {
            return FAIL;
          }
          tmp_path = path_finder_->getPath();
          shortenPath(tmp_path);
          start_pitch = camera_pitch;
          start_yaw = camera_yaw;
          end_pitch = hd_->final_pitch_[0];
          end_yaw = hd_->final_yaw_[0];
          start << pos(0), pos(1), pos(2), start_pitch, start_yaw;
          end << hd_->final_tour_[0](0), hd_->final_tour_[0](1), hd_->final_tour_[0](2), end_pitch,
              end_yaw;
          AngleInterpolation(start, end, tmp_path, updates_waypts);
          given_pitch.push_back(start_pitch);
          given_yaw.push_back(start_yaw);
        }
        else {
          path_finder_->reset();
          path_finder_->setResolution(0.5);
          if (path_finder_->search(hd_->final_tour_[i - 1], hd_->final_tour_[i]) !=
              Astar::REACH_END) {
            ROS_ERROR("%sdrone_%d No path to next viewpoint", color_map_[hp_->drone_id_].c_str(),
                hp_->drone_id_);
            return FAIL;
          }
          tmp_path = path_finder_->getPath();
          shortenPath(tmp_path);
          start_pitch = hd_->final_pitch_[i - 1];
          start_yaw = hd_->final_yaw_[i - 1];
          end_pitch = hd_->final_pitch_[i];
          end_yaw = hd_->final_yaw_[i];
          start << hd_->final_tour_[i - 1](0), hd_->final_tour_[i - 1](1),
              hd_->final_tour_[i - 1](2), start_pitch, start_yaw;
          end << hd_->final_tour_[i](0), hd_->final_tour_[i](1), hd_->final_tour_[i](2), end_pitch,
              end_yaw;
          AngleInterpolation(start, end, tmp_path, updates_waypts);
        }
        // hd_->path_next_goal_.insert(hd_->path_next_goal_.end(), tmp_path.begin(),
        // tmp_path.end()); avoid repeat path so i start with 1
        for (int j = 1; j < (int)updates_waypts.size(); j++) {
          Vector3d pos;
          pos << updates_waypts[j](0), updates_waypts[j](1), updates_waypts[j](2);
          waypts.push_back(pos);
          hd_->path_next_goal_.push_back(pos);
          given_pitch.push_back(updates_waypts[j](3));
          given_yaw.push_back(updates_waypts[j](4));
          if (j == (int)updates_waypts.size() - 1) {
            waypts_indexs.push_back(false);  // vp
          }
          else
            waypts_indexs.push_back(true);  // wps
        }
      }

      if (waypts.size() == 1)
        return NO_RESULT;

      Vector3d odom = pos;
      traj_generator_->HCTraj(odom, vel, acc, waypts, waypts_indexs, given_pitch, given_yaw);
      traj_generator_->getMincoPosTraj(
          hd_->minco_data_.minco_pos_traj_, hd_->minco_data_.minco_pos_anal_);
      traj_generator_->getMincoPitchTraj(
          hd_->minco_data_.minco_pitch_traj_, hd_->minco_data_.minco_pitch_anal_);
      traj_generator_->getMincoYawTraj(
          hd_->minco_data_.minco_yaw_traj_, hd_->minco_data_.minco_yaw_anal_);
      static int traj_id_ = 1;
      hd_->minco_data_.drone_id_ = hp_->drone_id_;
      hd_->minco_data_.start_time_ = ros::Time::now();
      hd_->minco_data_.traj_id_ = traj_id_;
      hd_->minco_data_.duration_ = hd_->minco_data_.minco_pos_traj_.getTotalDuration();
      traj_id_++;
    }
  }

  double total_time = (ros::Time::now() - t2).toSec();
  ROS_WARN(
      "%sdrone_%d Total time: %lf", color_map_[hp_->drone_id_].c_str(), hp_->drone_id_, total_time);
  ROS_ERROR_COND(total_time > 0.2, "drone_%d Total time too long!!!", hp_->drone_id_);

  return SUCCEED;
}

bool HeterogenousPlannerManager::checkTrajCollision()
{
  PlannerData* local_data_ = &hd_->minco_data_;
  double t_now = (ros::Time::now() - local_data_->start_time_).toSec();
  Eigen::Vector3d cur_pt = local_data_->minco_pos_traj_.getPos(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_->duration_) {
    fut_pt = local_data_->minco_pos_traj_.getPos(t_now + fut_t);
    if (sdf_map_->getInflateOccupancy(fut_pt) == 1) {
      std::cout << "collision at: " << fut_pt.transpose() << std::endl;
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }
  return true;
}

void HeterogenousPlannerManager::getAddAndDeleteViewpoints(const ViewpointsTask& drone_task,
    const ViewpointsTask& last_task, vector<VectorXd>& add_viewpoints,
    vector<VectorXd>& delete_viewpoints)
{
  vector<char> visited_last_flag;
  visited_last_flag.resize(last_task.points_.size(), -1);
  int no_action_num = 0;
  for (int i = 0; i < (int)drone_task.points_.size(); i++) {
    VectorXd now_viewpoint(5);
    now_viewpoint << drone_task.points_[i](0), drone_task.points_[i](1), drone_task.points_[i](2),
        drone_task.pitchs_[i], drone_task.yaws_[i];
    bool in_last_flag = false;
    for (int j = 0; j < (int)last_task.points_.size(); j++) {
      VectorXd last_viewpoint(5);
      last_viewpoint << last_task.points_[j](0), last_task.points_[j](1), last_task.points_[j](2),
          last_task.pitchs_[j], last_task.yaws_[j];
      if ((now_viewpoint - last_viewpoint).norm() < 0.001) {
        // no action
        in_last_flag = true;
        no_action_num++;
        visited_last_flag[j] = 1;
        break;
      }
    }
    // add
    if (!in_last_flag) {
      add_viewpoints.push_back(now_viewpoint);
    }
  }
  // delete
  for (int i = 0; i < (int)last_task.points_.size(); i++) {
    if (visited_last_flag[i] != -1)
      continue;
    VectorXd now_viewpoint(5);
    now_viewpoint << last_task.points_[i](0), last_task.points_[i](1), last_task.points_[i](2),
        last_task.pitchs_[i], last_task.yaws_[i];
    delete_viewpoints.push_back(now_viewpoint);
  }
}

/* photographers Global Planning */
// find a drone tsp tour
void HeterogenousPlannerManager::findViewpointsTour(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const double& cur_yaw, const double& cur_pitch,
    const vector<VectorXd>& viewpoints, vector<int>& indices)
{
  auto t1 = ros::Time::now();
  Eigen::MatrixXd cost_mat;
  VectorXd drone_pose(5);
  drone_pose << cur_pos(0), cur_pos(1), cur_pos(2), cur_pitch, cur_yaw;
  surface_coverage_->getFullViewpointsCostMatrix(drone_pose, viewpoints, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Initialize ATSP par file
  // Create problem file
  ofstream file(hp_->mtsp_dir_ + "/coverage_atsp_" + to_string(hp_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * cost_mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;
  file.open(hp_->mtsp_dir_ + "/coverage_atsp_" + to_string(hp_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + hp_->mtsp_dir_ + "/coverage_atsp_" + to_string(hp_->drone_id_) +
              ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + hp_->mtsp_dir_ + "/coverage_atsp_" + to_string(hp_->drone_id_) +
              ".tour\n";
  file.close();

  auto par_dir = hp_->mtsp_dir_ + "/coverage_atsp_" + to_string(hp_->drone_id_) + ".atsp";

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 2;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(hp_->mtsp_dir_ + "/coverage_atsp_" + to_string(hp_->drone_id_) + ".tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1)
      break;
    indices.push_back(id - 2);
  }

  res_file.close();

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("%s[findViewpointsTour] drone_%d Cost mat: %lf, TSP: %lf",
      color_map_[hp_->drone_id_].c_str(), hp_->drone_id_, mat_time, tsp_time);
}

/* explorer Task Assignment */
bool HeterogenousPlannerManager::viewpointsClustering(vector<VectorXd> viewpoints)
{
  static bool vp_cls_init = true;
  bool change_flag = false;
  if (vp_cls_init) {
    vp_cls_init = false;
    VAR.vp_clusters_.clear();
    VAR.vp_cluster_finder_.clear();
    VAR.vp_cluster_flag_.clear();
    VAR.cluster_num_ = 0;
  }

  vector<VectorXd> merge_viewpoints;
  for (int i = 0; i < (int)viewpoints.size(); i++) {
    if (VAR.vp_cluster_flag_[viewpoints[i]])
      continue;

    change_flag = true;
    Vector3d join_pos = viewpoints[i].head(3);
    int min_cluster_id = -1;
    for (int j = 0; j < (int)VAR.vp_clusters_.size(); j++) {
      if ((join_pos - VAR.vp_clusters_[j].average_).norm() > max_cluster_dist_)
        continue;

      bool join_flag = true;
      for (auto vp : VAR.vp_clusters_[j].viewpoints_) {
        Vector3d cluster_vp_pos = vp.head(3);
        if ((join_pos - cluster_vp_pos).norm() > max_cluster_dist_) {
          join_flag = false;
          break;
        }

        join_flag = raycastFunc(join_pos, cluster_vp_pos);
        if (!join_flag)
          break;
      }
      if (join_flag) {
        min_cluster_id = j;
        break;
      }
    }

    if (min_cluster_id != -1) {
      merge_viewpoints.push_back(viewpoints[i]);
    }
    // 自己变成新的一个类
    else {
      ViewpointCluster vp_cluster;
      VAR.vp_cluster_flag_[viewpoints[i]] = true;
      vp_cluster.cluster_id_ = VAR.cluster_num_;
      vp_cluster.average_ = viewpoints[i].head(3);
      vp_cluster.viewpoints_.push_back(viewpoints[i]);
      vp_cluster.cost_ = 0;
      VAR.vp_cluster_finder_[viewpoints[i]] = vp_cluster.cluster_id_;
      VAR.cluster_num_++;
      VAR.vp_clusters_.push_back(vp_cluster);
    }
  }

  for (int i = 0; i < (int)merge_viewpoints.size(); i++) {
    if (VAR.vp_cluster_flag_[merge_viewpoints[i]])
      continue;

    Vector3d join_pos = merge_viewpoints[i].head(3);
    int min_cluster_id = -1;
    double min_cluster_dist = 10000;
    for (int j = 0; j < (int)VAR.vp_clusters_.size(); j++) {
      if ((join_pos - VAR.vp_clusters_[j].average_).norm() > max_cluster_dist_)
        continue;

      bool join_flag = true;
      for (auto vp : VAR.vp_clusters_[j].viewpoints_) {
        Vector3d cluster_vp_pos = vp.head(3);
        if ((join_pos - cluster_vp_pos).norm() > max_cluster_dist_) {
          join_flag = false;
          break;
        }

        join_flag = raycastFunc(join_pos, cluster_vp_pos);
        if (!join_flag)
          break;
      }

      if (join_flag) {
        if ((join_pos - VAR.vp_clusters_[j].average_).norm() < min_cluster_dist) {
          min_cluster_id = j;
          min_cluster_dist = (join_pos - VAR.vp_clusters_[j].average_).norm();
        }
      }
    }

    if (min_cluster_id != -1) {
      VAR.vp_clusters_[min_cluster_id].viewpoints_.push_back(merge_viewpoints[i]);
      VAR.vp_clusters_[min_cluster_id].average_ =
          (VAR.vp_clusters_[min_cluster_id].average_ + merge_viewpoints[i].head(3)) / 2.0;
      // VAR.vp_clusters_[min_cluster_id].cost_ += (merge_viewpoints[i].head(3) -
      // VAR.vp_clusters_[min_cluster_id].average_).norm();
      VAR.vp_clusters_[min_cluster_id].cost_ += VAR.cluster_h_cost_;
      ROS_WARN("[Viewpoints Clustering] create a new cluster: id = %d cost_value = %f",
          min_cluster_id, VAR.vp_clusters_[min_cluster_id].cost_);
      VAR.vp_cluster_finder_[merge_viewpoints[i]] = VAR.vp_clusters_[min_cluster_id].cluster_id_;
      VAR.vp_cluster_flag_[merge_viewpoints[i]] = true;
    }
  }
  return change_flag;
}

void HeterogenousPlannerManager::findClustersDronesTour(const vector<ViewpointCluster>& clusters,
    const vector<VectorXd>& drones_pos, vector<vector<int>>& drone_indices)
{
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  getViewpointClustersMatrix(clusters, drones_pos, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();
  // ROS_WARN("MTSP Time = %f", mat_time);

  // Initialize ATSP par file
  // Create problem file
  ofstream file(hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".atsp");
  file << "NAME : amtsp3\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * cost_mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = drones_pos.size();
  file.open(hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  // file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_OBJECTIVE = MINMAX_SIZE\n";
  file << "MTSP_OBJECTIVE = MINMAX\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".atsp";

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 3;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }
  int drone_id = -1;
  drone_indices.clear();
  drone_indices.resize(drone_num);
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id > dimension)
      continue;
    if (id == -1)
      break;
    if (id - 1 <= drone_num) {
      drone_id = id - 2;
    }
    else {
      drone_indices[drone_id].push_back(id - drone_num - 2);
    }
  }
  res_file.close();

  // for (size_t i = 0; i < drone_indices.size(); ++i)
  // {
  //   std::cout << "drone_id = " << i << " ";
  //   for (size_t j = 0; j < drone_indices[i].size(); ++j)
  //   {
  //     std::cout << drone_indices[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }

  ROS_WARN(
      "Find cluster MTSP mat_time = %f mtsp_time = %f", mat_time, (ros::Time::now() - t1).toSec());
}

void HeterogenousPlannerManager::updateViewpointClustersMatrix()
{
  int last_dimension = VAR.last_cluster_num_;
  MatrixXd last_matrix = VAR.cluster_matrix_;

  int dimension = VAR.vp_clusters_.size();
  VAR.cluster_matrix_.resize(dimension, dimension);
  if (last_dimension > 0)
    VAR.cluster_matrix_.block(0, 0, last_dimension, last_dimension) = last_matrix;

  for (int i = VAR.last_cluster_num_; i < dimension; i++) {
    for (int j = 0; j < dimension; j++) {
      if (i == j) {
        VAR.cluster_matrix_(i, j) = 1000;
      }
      else {
        double cost =
            ViewNode::computePathLen(VAR.vp_clusters_[i].average_, VAR.vp_clusters_[j].average_);
        VAR.cluster_matrix_(i, j) = cost;
        VAR.cluster_matrix_(j, i) = cost;
      }
    }
  }
}

void HeterogenousPlannerManager::getViewpointClustersMatrix(
    const vector<ViewpointCluster>& clusters, const vector<VectorXd>& drones_pos,
    Eigen::MatrixXd& mat)
{
  // [Virtual depot + Drones + Clusters] Cost Matrix (MATSP)
  int cluster_num = clusters.size();
  int drone_num = drones_pos.size();
  int dimen = cluster_num + drone_num + 1;
  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = 0;
    mat(1 + i, 0) = 1000;
  }
  // Virtual depot to clusters
  for (int i = 0; i < cluster_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }

  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  // Costs from drones to clusters
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < cluster_num; j++) {
      double path_cost = ViewNode::computePathLen(drones_pos[i].head(3), clusters[j].average_);
      double point_cost = clusters[j].cost_;
      mat(1 + i, 1 + drone_num + j) = path_cost + point_cost;
      mat(1 + drone_num + j, 1 + i) = 10000;
    }
  }

  // Costs between clusters
  for (int i = 0; i < cluster_num; i++) {
    for (int j = i + 1; j < cluster_num; j++) {
      double path_cost = VAR.cluster_matrix_(i, j);
      // double path_cost = ViewNode::computePathLen(clusters[i].average_,
      // clusters[j].average_);
      mat(1 + drone_num + i, 1 + drone_num + j) = path_cost + clusters[j].cost_;
      mat(1 + drone_num + j, 1 + drone_num + i) = path_cost + clusters[i].cost_;
    }
  }

  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 10000;
  }
}

/* explorer Global Planning (TSP) */
void HeterogenousPlannerManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& indices)
{
  /* change ATSP to lhk3 */
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();
  for (int i = 0; i < dimension; i++) {
    cost_mat(i, i) = 1000;
  }

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Initialize ATSP par file
  // Create problem file
  ofstream file(hp_->mtsp_dir_ + "/exploration_atsp_" + to_string(hp_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * cost_mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;
  file.open(hp_->mtsp_dir_ + "/exploration_atsp_" + to_string(hp_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + hp_->mtsp_dir_ + "/exploration_atsp_" + to_string(hp_->drone_id_) +
              ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_MIN_SIZE = " << to_string(min(int(hd_->frontiers_.size()) / drone_num, 4)) <<
  // "\n"; file << "MTSP_MAX_SIZE = "
  //      << to_string(max(1, int(hd_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + hp_->mtsp_dir_ + "/exploration_atsp_" + to_string(hp_->drone_id_) +
              ".tour\n";
  file.close();

  auto par_dir = hp_->mtsp_dir_ + "/exploration_atsp_" + to_string(hp_->drone_id_) + ".atsp";

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 1;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(hp_->mtsp_dir_ + "/exploration_atsp_" + to_string(hp_->drone_id_) + ".tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1)
      break;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, hd_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("%s[Exploration Global Tour] drone_%d Cost mat: %lf, TSP: %lf",
      color_map_[hp_->drone_id_].c_str(), hp_->drone_id_, mat_time, tsp_time);
}
/* Traj generation for next path */
void HeterogenousPlannerManager::shortenPath(vector<Vector3d>& path)
{
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 2.0;
  vector<Vector3d> short_tour = { path.front() };
  for (size_t i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3)
    short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2 && hp_->drone_id_ == 0)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void HeterogenousPlannerManager::AngleInterpolation(const Eigen::VectorXd& start,
    const Eigen::VectorXd& end, const vector<Eigen::Vector3d>& waypts_,
    vector<Eigen::VectorXd>& updates_waypts_)
{
  updates_waypts_.clear();
  int dof = start.size(), angle = dof - 3;
  double dist_gap = 0.0;
  double whole_dist = 0.0;
  vector<Eigen::VectorXd> effect_vps_;
  vector<Eigen::VectorXd> seg_posi_;
  double yaw_start = 0.0, yaw_end = 0.0, pitch_start = 0.0, pitch_end = 0.0;
  int cal_flag_yaw = 0, cal_flag_pitch = 0;
  int cal_dir_yaw = 0, cal_dir_pitch = 0;

  if (angle == 1) {
    yaw_start = start(3) * 180.0 / M_PI;
    yaw_end = end(3) * 180.0 / M_PI;
    cal_flag_yaw = yaw_end - yaw_start > 0 ? 1 : -1;
    cal_dir_yaw = abs(yaw_end - yaw_start) > 180.0 ? -1 : 1;
  }
  else {
    yaw_start = start(4) * 180.0 / M_PI;
    yaw_end = end(4) * 180.0 / M_PI;
    cal_flag_yaw = yaw_end - yaw_start > 0 ? 1 : -1;
    cal_dir_yaw = abs(yaw_end - yaw_start) > 180.0 ? -1 : 1;
    pitch_start = start(3) * 180.0 / M_PI;
    pitch_end = end(3) * 180.0 / M_PI;
    cal_flag_pitch = ((pitch_end - pitch_start) > 0) ? 1 : -1;
    cal_dir_pitch = abs(pitch_end - pitch_start) > 180.0 ? -1 : 1;
  }

  effect_vps_.push_back(start);
  for (auto x : waypts_) {
    if ((x - start.head(3)).norm() >= dist_gap && (x - end.head(3)).norm() >= dist_gap) {
      Eigen::VectorXd aug_x;
      aug_x.resize(dof);
      aug_x(0) = x(0);
      aug_x(1) = x(1);
      aug_x(2) = x(2);
      effect_vps_.push_back(aug_x);
    }
  }
  effect_vps_.push_back(end);

  seg_posi_.push_back(start);
  seg_posi_.insert(seg_posi_.end(), effect_vps_.begin(), effect_vps_.end());
  seg_posi_.push_back(end);
  for (int i = 0; i < (int)seg_posi_.size() - 1; ++i)
    whole_dist += (seg_posi_[i + 1].head(3) - seg_posi_[i].head(3)).norm();

  if ((int)effect_vps_.size() > 0) {
    double yaw_gap = abs(yaw_end - yaw_start) > 180.0 ? (360.0 - abs(yaw_end - yaw_start)) :
                                                        abs(yaw_end - yaw_start);
    if (angle == 1) {
      for (int i = 1; i < (int)effect_vps_.size() - 1; ++i) {
        double e_dist = (effect_vps_[i].head(3) - start.head(3)).norm();
        effect_vps_[i](3) =
            (yaw_start + cal_dir_yaw * cal_flag_yaw * yaw_gap * e_dist / whole_dist) * M_PI / 180.0;
        while (effect_vps_[i](3) < -M_PI) effect_vps_[i](3) += 2 * M_PI;
        while (effect_vps_[i](3) > M_PI) effect_vps_[i](3) -= 2 * M_PI;

        updates_waypts_.push_back(effect_vps_[i]);
      }
    }
    else {
      double pitch_gap = abs(pitch_end - pitch_start) > 180.0 ?
                             (360.0 - abs(pitch_end - pitch_start)) :
                             abs(pitch_end - pitch_start);
      for (int i = 1; i < (int)effect_vps_.size() - 1; ++i) {
        double e_dist = (effect_vps_[i].head(3) - start.head(3)).norm();
        effect_vps_[i](3) =
            (pitch_start + cal_dir_pitch * cal_flag_pitch * pitch_gap * e_dist / whole_dist) *
            M_PI / 180.0;
        effect_vps_[i](4) =
            (yaw_start + cal_dir_yaw * cal_flag_yaw * yaw_gap * e_dist / whole_dist) * M_PI / 180.0;
        while (effect_vps_[i](3) < -M_PI) effect_vps_[i](3) += 2 * M_PI;
        while (effect_vps_[i](3) > M_PI) effect_vps_[i](3) -= 2 * M_PI;
        while (effect_vps_[i](4) < -M_PI) effect_vps_[i](4) += 2 * M_PI;
        while (effect_vps_[i](4) > M_PI) effect_vps_[i](4) -= 2 * M_PI;

        updates_waypts_.push_back(effect_vps_[i]);
      }
    }
  }
}

void HeterogenousPlannerManager::findClustersDronesTourV2(const vector<VectorXd>& drones_pos,
    const vector<int>& existing_cluster_indices, vector<vector<int>>& drone_indices)
{
  auto t1 = ros::Time::now();
  drone_indices.clear();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  getMTSPCostMatrixV2(drones_pos, existing_cluster_indices, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();
  // ROS_WARN("MTSP Time = %f", mat_time);

  // Initialize ATSP par file
  // Create problem file
  ofstream file(hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".atsp");
  file << "NAME : amtsp3\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * cost_mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = drones_pos.size();
  file.open(hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  // file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_OBJECTIVE = MINMAX_SIZE\n";
  file << "MTSP_OBJECTIVE = MINMAX\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".atsp";

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 3;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(hp_->mtsp_dir_ + "/amtsp3_" + to_string(hp_->drone_id_) + ".tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }
  int drone_id = -1;
  drone_indices.clear();
  drone_indices.resize(drone_num);
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id > dimension)
      continue;
    if (id == -1)
      break;
    if (id - 1 <= drone_num) {
      drone_id = id - 2;
    }
    else {
      drone_indices[drone_id].push_back(id - drone_num - 2);
    }
  }
  res_file.close();

  for (size_t i = 0; i < drone_indices.size(); ++i) {
    std::stringstream ss;
    ss << "drone_id = " << i << ": ";
    for (size_t j = 0; j < drone_indices[i].size(); ++j) ss << drone_indices[i][j] << " ";
    ROS_WARN("%s", ss.str().c_str());
  }

  ROS_WARN("[Task Assignment] Find cluster MTSPV2 mat_time = %f mtsp_time = %f", mat_time,
      (ros::Time::now() - t1).toSec());
}

/* MDMTSP_GA */
bool HeterogenousPlannerManager::taskAssignment(const vector<VectorXd>& drones_pos,
    const vector<int>& existing_cluster_indices,
    const vector<vector<int>>& last_drone_existing_cluster_ids, vector<vector<int>>& drone_indices,
    const vector<int>& last_cluster_ids, const vector<int>& new_cluster_ids)
{
  if (existing_cluster_indices.size() == 0)
    return false;

  // print last task assignment result
  ROS_WARN("[Task Assignment] all drones last tour:");
  for (int i = 0; i < (int)last_drone_existing_cluster_ids.size(); i++) {
    std::stringstream ss;
    ss << "[Task Assignment] drone_" << i + 1 << " last tour: ";
    for (auto idx : last_drone_existing_cluster_ids[i]) ss << idx << " ";
    ROS_WARN("%s", ss.str().c_str());
  }

  Eigen::MatrixXd dmat, D0, D1;
  getMdmtspCostMatrix(drones_pos, existing_cluster_indices, dmat, D0, D1);
  // ROS_WARN_STREAM("[Task Assignment] D0 mat is:" << D0);
  int iterations = mdmtsp_iterations_;
  if (final_task_assignment_)
    iterations = mdmtsp_iterations_ * 2;
  vector<vector<int>> tmp_indices;
  MDMTSP_GA(dmat, D0, D1, iterations, tmp_indices, last_drone_existing_cluster_ids,
      last_cluster_ids, new_cluster_ids);

  // print task assignment result
  drone_indices.clear();
  drone_indices.assign(tmp_indices.begin(), tmp_indices.end());
  ROS_WARN("[Task Assignment] all drones current cluster tour:");
  for (int i = 0; i < (int)drone_indices.size(); i++) {
    std::stringstream ss;
    ss << "[Task Assignment] drone_" << i + 1 << ": current cluster tour: ";
    for (auto idx : drone_indices[i]) ss << idx << " ";
    ROS_WARN("%s", ss.str().c_str());
  }

  return true;
}

void HeterogenousPlannerManager::MDMTSP_GA(const MatrixXd& dmat, const MatrixXd& D0,
    const MatrixXd& D1, int max_iter, vector<vector<int>>& drone_indices,
    const vector<vector<int>>& last_drone_existing_cluster_ids, const vector<int>& last_cluster_ids,
    const vector<int>& new_cluster_ids)
{
  auto t1 = ros::Time::now();
  int max_salesmen = D0.rows();
  int city_num = D0.cols();
  int cost_type = 2;  // 1 MINSUM 2 MINMAX
  int pop_size = 60;

  int n = city_num;

  // Initialize populations
  MatrixXi pop_rte(pop_size, n);          // Population of routes
  vector<vector<int>> pop_brk(pop_size);  // Population of breaks
  for (int k = 0; k < pop_size; ++k) {
    if (k % 3 == 0) {
      VectorXi perm = VectorXi::LinSpaced(n, 0, n - 1).eval();  // 城市0——n-1的排列
      std::random_shuffle(perm.data(), perm.data() + perm.size());
      pop_rte.row(k) = perm;
      pop_brk[k] = randBreakIdx(max_salesmen, n);  // 生成范围是 0~n-1 的 max_salesmen-1 个分割index
    }
    else {
      // get population init value
      VectorXi init_p_rte;
      vector<int> init_p_brk;
      getMdmtspInitValue(last_drone_existing_cluster_ids, last_cluster_ids, new_cluster_ids,
          init_p_rte, init_p_brk);
      pop_rte.row(k) = init_p_rte;
      pop_brk[k] = init_p_brk;
    }
  }

  double global_min = numeric_limits<double>::infinity();
  VectorXi opt_rte;
  vector<int> opt_brk;
  int best_generation_iter = 0;
  int salesmen = 0;
  double epsilon = 1e-5;
  MatrixXi new_pop_rte(pop_size, n);
  vector<vector<int>> new_pop_brk(pop_size);

  for (int i = 0; i < max_iter; i++) {
    // Evaluate each population member
    vector<double> calc_cost(pop_size);
    for (int p = 0; p < pop_size; ++p) {
      vector<double> cost_list;
      VectorXi p_rte = pop_rte.row(p);
      vector<int> p_brk = pop_brk[p];
      salesmen = p_brk.size() + 1;
      MatrixXi rng = calcRange(p_brk, n);
      for (int sa = 0; sa < salesmen; ++sa) {
        if (rng(sa, 0) <= rng(sa, 1)) {
          vector<int> Tour;
          Tour.push_back(sa);
          for (int i = rng(sa, 0); i <= rng(sa, 1); ++i) {
            Tour.push_back(p_rte(i));
          }
          Tour.push_back(sa);

          double cost = 0;
          double len_cost = calculateTourLength(Tour, dmat, D0, D1);
          double consistent_cost =
              calculateTourConsistent(sa, Tour, last_drone_existing_cluster_ids, dmat, D0);
          if (!final_task_assignment_)
            cost = len_cost + consistent_cost;
          else
            cost = len_cost;
          cost_list.push_back(cost);
        }
        else {
          vector<int> Tour = { sa, sa };
          cost_list.push_back(
              100000);  // Set a maximum value to prevent any aircraft from having no tasks.
        }
      }

      double total = 0.0;
      for (double cs : cost_list) total += cs;
      if (cost_type == 1) {
        calc_cost[p] = total;
      }
      else if (cost_type == 2) {
        double max_cost = *max_element(cost_list.begin(), cost_list.end());
        calc_cost[p] = max_cost + epsilon * total;
      }
    }

    // Find the Best Route in the Population
    double min_dist = *min_element(calc_cost.begin(), calc_cost.end());
    if (min_dist < global_min) {
      best_generation_iter = i;
      global_min = min_dist;
      int index = min_element(calc_cost.begin(), calc_cost.end()) - calc_cost.begin();
      opt_rte = pop_rte.row(index);
      opt_brk = pop_brk[index];
    }

    // Genetic Algorithm Operators
    vector<int> rand_grouping = randperm(pop_size);
    int ops = 12;

    for (int p = ops; p <= pop_size; p += ops) {
      MatrixXi rtes(ops, n);
      vector<vector<int>> brks(ops);
      vector<double> dists(ops);
      for (int i = 0; i < ops; ++i) {
        rtes.row(i) = pop_rte.row(rand_grouping[p - ops + i]);
        brks[i] = pop_brk[rand_grouping[p - ops + i]];
        dists[i] = calc_cost[rand_grouping[p - ops + i]];
      }

      vector<double>::iterator min_iter = min_element(dists.begin(), dists.end());
      int idx = distance(dists.begin(), min_iter);
      VectorXi best_of_8_rte = rtes.row(idx);
      vector<int> best_of_8_brk = brks[idx];
      VectorXi rte_ins_pts(2);
      VectorXi indices = VectorXi::LinSpaced(n, 0, n - 1);
      std::shuffle(indices.data(), indices.data() + n, std::mt19937(std::random_device()()));
      rte_ins_pts(0) = indices(0);
      rte_ins_pts(1) = indices(1);
      int I = min(rte_ins_pts(0), rte_ins_pts(1));
      int J = max(rte_ins_pts(0), rte_ins_pts(1));
      MatrixXi tmp_pop_rte = MatrixXi::Constant(ops, n, 0);
      vector<vector<int>> tmp_pop_brk(ops);
      for (int k = 0; k < ops; ++k) {
        tmp_pop_rte.row(k) = best_of_8_rte;
        tmp_pop_brk[k] = best_of_8_brk;

        switch (k) {
          case 1:  // Flip
          {
            int II = I, JJ = J;
            while (II < JJ) {
              std::swap(tmp_pop_rte(k, II), tmp_pop_rte(k, JJ));
              ++II;
              --JJ;
            }
          } break;
          case 2:  // Swap
          {
            swap(tmp_pop_rte(k, I), tmp_pop_rte(k, J));
          } break;
          case 3:  // Slide
          {
            int tmp = tmp_pop_rte(k, J);
            for (int l = J; l > I; --l) {
              tmp_pop_rte(k, l) = tmp_pop_rte(k, l - 1);
            }
            tmp_pop_rte(k, I) = tmp;
          } break;
          case 4:  // Change Breaks
            tmp_pop_brk[k] = randBreakIdx(max_salesmen, n);
            break;
          case 5:  // Flip, Change Breaks
          {
            int II = I, JJ = J;
            while (II < JJ) {
              std::swap(tmp_pop_rte(k, II), tmp_pop_rte(k, JJ));
              ++II;
              --JJ;
            }
            tmp_pop_brk[k] = randBreakIdx(max_salesmen, n);
          } break;
          case 6:  // Swap, Change Breaks
          {
            swap(tmp_pop_rte(k, I), tmp_pop_rte(k, J));
            tmp_pop_brk[k] = randBreakIdx(max_salesmen, n);
          } break;
          case 7:  // Slide, Change Breaks
          {
            int tmp = tmp_pop_rte(k, J);
            for (int l = J; l > I; --l) {
              tmp_pop_rte(k, l) = tmp_pop_rte(k, l - 1);
            }
            tmp_pop_rte(k, I) = tmp;
            tmp_pop_brk[k] = randBreakIdx(max_salesmen, n);
          } break;
          case 8: {
            if (min(J - I, min(n - J, static_cast<int>(floor(sqrt(n))))) >= 1) {
              std::random_device rd;
              std::mt19937 gen(rd());
              std::uniform_int_distribution<int> dis(
                  1, min(J - I, min(n - J, static_cast<int>(floor(sqrt(n))))));
              int l = dis(gen);
              VectorXi temp1 = tmp_pop_rte.row(k).segment(I, l);
              VectorXi temp2 = tmp_pop_rte.row(k).segment(J, l);
              tmp_pop_rte.row(k).segment(I, l) = temp2;
              tmp_pop_rte.row(k).segment(J, l) = temp1;
            }
          } break;
          case 9: {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(1, max_salesmen - 1);
            int l = dis(gen);
            vector<int> temp = tmp_pop_brk[k];
            temp.erase(temp.begin() + l - 1);
            temp.push_back(n - 1);
            tmp_pop_brk[k] = temp;
          } break;
          case 10: {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(1, max_salesmen - 1);
            int l = dis(gen);
            vector<int> temp = tmp_pop_brk[k];
            temp.erase(temp.begin() + l - 1);
            temp.insert(temp.begin(), 0);
            tmp_pop_brk[k] = temp;
          } break;
          case 11: {
          } break;
          default: {        // Swap close points
            if (I < n - 1)  // n or n-1
            {
              swap(tmp_pop_rte(k, I), tmp_pop_rte(k, I + 1));
            }
          } break;
        }
      }

      for (int kk = 0; kk < ops; ++kk) {
        new_pop_rte.block(p - ops + kk, 0, 1, n) = tmp_pop_rte.row(kk);
        new_pop_brk[p - ops + kk] = tmp_pop_brk[kk];
      }
    }

    pop_rte = new_pop_rte;
    pop_brk = new_pop_brk;
  }

  drone_indices.clear();
  drone_indices.resize(max_salesmen);
  // ROS_WARN_STREAM("opt pop :" << opt_rte.transpose());
  MatrixXi rng = calcRange(opt_brk, n);
  // ROS_WARN_STREAM("rng :" << rng);
  for (int sa = 0; sa < max_salesmen; ++sa) {
    if (rng(sa, 0) <= rng(sa, 1)) {
      vector<int> Tour;
      Tour.push_back(sa);
      for (int i = rng(sa, 0); i <= rng(sa, 1); ++i) {
        Tour.push_back(opt_rte(i));
        drone_indices[sa].push_back(opt_rte(i));
      }
      Tour.push_back(sa);

      double cost = 0;
      double len_cost = calculateTourLength(Tour, dmat, D0, D1);
      double consistent_cost =
          calculateTourConsistent(sa, Tour, last_drone_existing_cluster_ids, dmat, D0);
      if (!final_task_assignment_)
        cost = len_cost + consistent_cost;
      else
        cost = len_cost;
      ROS_WARN(
          "[Consistent-MDMTSP] Tour salesman %d len_cost is %f all_cost = %f", sa, len_cost, cost);
    }
  }

  ROS_WARN("[Consistent-MDMTSP] iter = %d Now Best Cost = %f", best_generation_iter, global_min);
  ROS_WARN("[Consistent-MDMTSP] totol time is %f", (ros::Time::now() - t1).toSec());
}

void HeterogenousPlannerManager::getMTSPCostMatrixV2(const vector<VectorXd>& drones_pos,
    const vector<int>& existing_cluster_indices, Eigen::MatrixXd& cost_mat)
{
  int drone_num = drones_pos.size();
  int existing_cluster_num = existing_cluster_indices.size();

  int dimen = 1 + drone_num + existing_cluster_num;
  cost_mat = Eigen::MatrixXd::Constant(dimen, dimen, 100000.0);

  for (int i = 0; i < drone_num; i++) {
    cost_mat(0, 1 + i) = 0;
  }

  for (int i = 0; i < existing_cluster_num; i++) {
    cost_mat(1 + drone_num + i, 0) = 0;
  }

  // drones to useful_cluster
  for (int i = 0; i < drone_num; i++) {
    for (int j = 0; j < existing_cluster_num; j++) {
      int cls_id = existing_cluster_indices[j];
      double path_cost =
          ViewNode::computePathLen(drones_pos[i].head(3), VAR.vp_clusters_[cls_id].average_);
      double point_cost = VAR.vp_clusters_[cls_id].cost_;
      cost_mat(1 + i, 1 + drone_num + j) = path_cost + point_cost;
      // cost_mat(1+drone_num + j, i) = 0.0;
    }
  }

  for (int i = 0; i < existing_cluster_num; i++) {
    int cls_id1 = existing_cluster_indices[i];
    for (int j = i + 1; j < existing_cluster_num; j++) {
      int cls_id2 = existing_cluster_indices[j];
      double path_cost = VAR.cluster_matrix_(cls_id1, cls_id2);
      double point_cost1 = VAR.vp_clusters_[cls_id1].cost_;
      double point_cost2 = VAR.vp_clusters_[cls_id2].cost_;
      cost_mat(1 + drone_num + i, 1 + drone_num + j) = path_cost + point_cost2;
      cost_mat(1 + drone_num + j, 1 + drone_num + i) = path_cost + point_cost1;
    }
  }
}

void HeterogenousPlannerManager::getMdmtspCostMatrix(const vector<VectorXd>& drones_pos,
    const vector<int>& existing_cluster_indices, Eigen::MatrixXd& dmat, Eigen::MatrixXd& D0,
    Eigen::MatrixXd& D1)
{
  MatrixXd cost_mat;
  int drone_num = drones_pos.size();
  int existing_cluster_num = existing_cluster_indices.size();

  int dimen = drone_num + existing_cluster_num;
  cost_mat = Eigen::MatrixXd::Constant(dimen, dimen, 100000.0);

  // drones to useful_cluster
  for (int i = 0; i < drone_num; i++) {
    for (int j = 0; j < existing_cluster_num; j++) {
      int cls_id = existing_cluster_indices[j];
      double path_cost =
          ViewNode::computePathLen(drones_pos[i].head(3), VAR.vp_clusters_[cls_id].average_);
      // if (path_cost >= 999) {
      //   ROS_WARN("A_star not find??");
      // }
      double point_cost = VAR.vp_clusters_[cls_id].cost_;
      cost_mat(i, drone_num + j) = path_cost + point_cost;
      cost_mat(drone_num + j, i) = 0.0;
    }
  }

  for (int i = 0; i < existing_cluster_num; i++) {
    int cls_id1 = existing_cluster_indices[i];
    for (int j = i + 1; j < existing_cluster_num; j++) {
      int cls_id2 = existing_cluster_indices[j];
      double path_cost = VAR.cluster_matrix_(cls_id1, cls_id2);
      double point_cost1 = VAR.vp_clusters_[cls_id1].cost_;
      double point_cost2 = VAR.vp_clusters_[cls_id2].cost_;
      cost_mat(drone_num + i, drone_num + j) = path_cost + point_cost2;
      cost_mat(drone_num + j, drone_num + i) = path_cost + point_cost1;
    }
  }

  dmat = cost_mat.block(drone_num, drone_num, existing_cluster_num, existing_cluster_num);
  D0 = cost_mat.block(0, drone_num, drone_num, existing_cluster_num);
  D1 = cost_mat.block(drone_num, 0, existing_cluster_num, drone_num);
}

void HeterogenousPlannerManager::getMdmtspInitValue(
    const vector<vector<int>>& last_drone_existing_cluster_ids, const vector<int>& last_cluster_ids,
    const vector<int>& new_cluster_ids, VectorXi& p_rte, vector<int>& p_brk)
{
  // get population init value

  p_brk.clear();
  // Insert new_cluster_ids into last_cluster_ids to get the new init_value.
  vector<int> init_value = randomInsert(last_cluster_ids, new_cluster_ids);
  vector<int> init_brk;
  int cur_index = 0;
  int sub_drone_id = 0;
  int sub_index = 0;
  for (auto init_v : init_value) {
    cur_index++;
    // ROS_WARN("cur_index = %d init value = %d ", cur_index, init_v);
    while (sub_drone_id < (int)last_drone_existing_cluster_ids.size() &&
           last_drone_existing_cluster_ids[sub_drone_id].size() == 0) {
      sub_drone_id++;
      init_brk.push_back(cur_index - 1);
    }
    if (sub_drone_id == (int)last_drone_existing_cluster_ids.size())
      break;
    if (init_v == last_drone_existing_cluster_ids[sub_drone_id][sub_index]) {
      sub_index++;
      if (sub_index == (int)last_drone_existing_cluster_ids[sub_drone_id].size()) {
        sub_index = 0;
        sub_drone_id++;
        if (sub_drone_id == (int)last_drone_existing_cluster_ids.size())
          break;
        init_brk.push_back(cur_index - 1);
        // ROS_WARN("break value = %d", cur_index - 1);
      }
    }
  }
  Eigen::Map<Eigen::VectorXi> tmp_rte(init_value.data(), init_value.size());

  p_brk = init_brk;
  p_rte = tmp_rte;
}

vector<int> HeterogenousPlannerManager::randperm(int n)
{
  vector<int> perm(n);
  iota(perm.begin(), perm.end(), 0);
  random_shuffle(perm.begin(), perm.end());
  return perm;
}

std::vector<int> HeterogenousPlannerManager::randBreakIdx(int max_salesmen, int n)
{
  int num_brks = max_salesmen - 1;
  std::vector<int> breaks(num_brks);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0, n - 1);
  for (int i = 0; i < num_brks; ++i) {
    breaks[i] = dis(gen);
  }
  std::sort(breaks.begin(), breaks.end());
  return breaks;
}

double HeterogenousPlannerManager::calculateTourLength(
    const vector<int>& Tour, const MatrixXd& dmat, const MatrixXd& D0, const MatrixXd& D1)
{
  double VehicleTourLength = D0(Tour[0], Tour[1]);
  for (int c = 1; c < (int)Tour.size() - 2; ++c) VehicleTourLength += dmat(Tour[c], Tour[c + 1]);
  if (Tour.size() > 2)
    VehicleTourLength += D1(Tour[Tour.size() - 2], Tour[Tour.size() - 1]);
  return VehicleTourLength;
}

// Function to calculate the range of each salesman's route
MatrixXi HeterogenousPlannerManager::calcRange(const vector<int>& p_brk, int n)
{
  MatrixXi rng(p_brk.size() + 1, 2);
  int flag = 1;
  for (int i = 0; i < (int)p_brk.size(); ++i) {
    if (flag == 1 && p_brk[i] > 0) {
      rng(i, 0) = 0;
      rng(i, 1) = p_brk[i];
      flag = 0;
    }
    else if (flag == 1) {
      rng(i, 0) = 0;
      rng(i, 1) = -1;
    }
    else if (p_brk[i] <= p_brk[i - 1]) {
      rng(i, 0) = p_brk[i - 1];
      rng(i, 1) = p_brk[i];
    }
    else if (i < (int)p_brk.size() - 1) {
      rng(i, 0) = p_brk[i - 1] + 1;
      rng(i, 1) = p_brk[i];
    }
    else {
      rng(i, 0) = p_brk[i - 1] + 1;
      rng(i, 1) = p_brk[i];
    }
  }

  if (p_brk[p_brk.size() - 1] < n - 1 && p_brk[p_brk.size() - 1] != 0) {
    rng(p_brk.size(), 0) = p_brk[p_brk.size() - 1] + 1;
    rng(p_brk.size(), 1) = n - 1;
  }
  else if (p_brk[p_brk.size() - 1] < n - 1 && p_brk[p_brk.size() - 1] == 0) {
    rng(p_brk.size(), 0) = p_brk[p_brk.size() - 1];
    rng(p_brk.size(), 1) = n - 1;
  }
  else {
    rng(p_brk.size(), 0) = p_brk[p_brk.size() - 1];
    rng(p_brk.size(), 1) = n - 2;
  }
  // ROS_WARN_STREAM("range matrix:");
  // ROS_WARN_STREAM(rng);
  return rng;
}

double HeterogenousPlannerManager::calculateTourConsistent(int drone_id, const vector<int>& Tour,
    const vector<vector<int>>& last_drone_existing_cluster_ids, const MatrixXd& dmat,
    const MatrixXd& D0)
{
  if (last_drone_existing_cluster_ids[drone_id].size() == 0 || Tour.size() == 2)
    return 0.0;
  for (int i = 0; i < (int)last_drone_existing_cluster_ids.size(); i++) {
    if (last_drone_existing_cluster_ids[i].size() == 0)
      return 0.0;
  }

  double consistent = 0.0;
  double dist = 0.0;
  for (int i = 1; i < (int)Tour.size() - 1; ++i) {
    if (i - 1 >= (int)last_drone_existing_cluster_ids[drone_id].size())
      break;
    if (i == 1)
      dist += D0(drone_id, Tour[1]) * 0.1;
    else
      dist += dmat(Tour[i - 1], Tour[i]);

    if (last_drone_existing_cluster_ids[drone_id][i - 1] == Tour[i])
      consistent -= consistent_value_ * exp(-consistent_alpha_ * dist);
    else
      break;
  }
  return consistent;
}

}  // namespace hetero_planner
