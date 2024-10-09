#ifndef _HETERO_DATA_H_
#define _HETERO_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <gcopter/minco.hpp>

using Eigen::Vector3d;
using std::vector;

namespace hetero_planner {
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;
  double odom_camera_yaw_;
  double odom_camera_pitch_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  double dis_to_next_vp;
  double camera_pitch;
  double camera_yaw;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second
};

struct DroneState {
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double pitch_;
  double yaw_;
  double stamp_;                 // Stamp of pos,vel,yaw
  double recent_attempt_time_;   // Stamp of latest opt attempt with any drone
  double recent_interact_time_;  // Stamp of latest opt with this drone
};

struct ViewpointsTask {
  int drone_id_;
  double stamp_;
  double recent_attempt_time_;
  vector<int> orders_;
  vector<Vector3d> points_;
  vector<double> pitchs_;
  vector<double> yaws_;
};

struct PlannerData {
  int traj_id_;
  int drone_id_;
  ros::Time start_time_;
  int duration_;
  Trajectory<7> minco_pos_traj_, minco_picth_traj_, minco_yaw_traj_, minco_pitch_traj_;
  minco::MINCO_S4NU minco_pos_anal_, minco_pitch_anal_, minco_yaw_anal_;
};

struct HeteroData {
  vector<vector<Vector3d>> frontiers_, surfaces_, surface_frontiers_;
  vector<vector<Vector3d>> dead_frontiers_, dead_surfaces_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_, surface_points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_, surface_views_;
  vector<double> yaws_, surface_yaws_, surface_pitchs_;
  vector<Vector3d> global_tour_;
  vector<char> surface_states_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;

  // viewpoint planning
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;

  // final viewpoints
  vector<Vector3d> updated_points_, updated_views_, updated_visib_cells_, all_points_;
  vector<int> updated_counts_, updated_iternums_;
  vector<int> updated_ids_;
  vector<Vector3d> updated_views1_, updated_views2_;

  // viewpoints cluster
  vector<Vector3d> cluster_viewpoints_;
  vector<Vector3d> cluster_averages_;
  vector<Vector3d> boundary_clusters_, all_boundary_clusters_;
  vector<vector<Vector3d>> drone_cluster_averages_;
  vector<int> cluster_ids_;
  int cluster_num_;

  // final viewpoint tour
  vector<Vector3d> final_tour_;
  vector<double> final_pitch_;
  vector<double> final_yaw_;
  vector<int> global_id_;
  unordered_map<VectorXd, int> visited_tour_; // [pose(x,y,z,pitch,yaw), is_visited]
  vector<VectorXd> last_visited_vps_;         // last iteration visited viewpoints
  PlannerData minco_data_;

  // Swarm, other drones' state
  vector<DroneState> swarm_state_;
  vector<ViewpointsTask> swarm_task_;
  ViewpointsTask last_self_task_;

  ros::Time surface_update_time_ = ros::Time::now();
};

struct HeteroParam {
  // params
  string mtsp_dir_;  // resource dir of tsp solver

  int drone_id_;
  int drone_num_;
};

}  // namespace hetero_planner

#endif