#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <iostream>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace hetero_planner {
class PerceptionUtils {
public:
  PerceptionUtils(ros::NodeHandle& nh, int mission_type);
  ~PerceptionUtils()
  {
  }
  // Set position and yaw
  void setPose(const Vector3d& pos, const double& yaw);
  void setPose_PY(const Vector3d& pos, const double& pitch, const double& yaw);

  // Get info of current pose
  void getFOV(vector<Vector3d>& list1, vector<Vector3d>& list2);
  void getFOV_PY(vector<Vector3d>& list1, vector<Vector3d>& list2);
  bool insideFOV(const Vector3d& point);
  void getFOVBoundingBox(Vector3d& bmin, Vector3d& bmax);

private:
  // Data
  // Current camera pos and yaw
  Vector3d pos_;
  double yaw_, pitch_;
  // Camera plane's normals in world frame
  vector<Vector3d> normals_;

  /* Params */
  // Sensing range of camera
  double max_dist_, vis_dist_, left_angle_, right_angle_, top_angle_, bottom_angle_;
  double lidar_max_dist_, lidar_vis_dist_, lidar_left_angle_, lidar_right_angle_, lidar_top_angle_,
      lidar_bottom_angle_;
  double camera_max_dist_, camera_vis_dist_, camera_left_angle_, camera_right_angle_,
      camera_top_angle_, camera_bottom_angle_;
  double lidar_pitch_;
  // Normal vectors of camera FOV planes in camera frame
  Vector3d n_top_, n_bottom_, n_left_, n_right_;
  // Transform between camera and body
  Eigen::Matrix4d T_cb_, T_bc_;
  // FOV vertices in body frame
  vector<Vector3d> cam_vertices1_, cam_vertices2_;
};

}  // namespace hetero_planner
#endif