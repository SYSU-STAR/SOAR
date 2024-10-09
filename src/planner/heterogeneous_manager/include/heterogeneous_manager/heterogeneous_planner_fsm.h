#ifndef _HETEROGENOUS_PLANNER_FSM_H_
#define _HETEROGENOUS_PLANNER_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <quadrotor_msgs/PolyTraj.h>

#include <heterogeneous_manager/DroneState.h>
#include <heterogeneous_manager/ViewpointsTask.h>
#include <heterogeneous_manager/VisitedTour.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace hetero_planner {
class HeterogenousPlannerManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum FSM_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class HeterogeneousPlannerFSM {
private:
  /* planning utils */
  shared_ptr<HeterogenousPlannerManager> hetero_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  FSM_STATE state_;

  double resolution_;
  int drone_num_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, minco_traj_pub_;

  /* helper functions */
  int callHeterogeneousPlanner();
  void transitState(FSM_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();
  int getSlfDroneId();
  int isExplorer();
  void polyTraj2ROSMsg(quadrotor_msgs::PolyTraj& poly_msg);

  /* Swarm */
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const heterogeneous_manager::DroneStateConstPtr& msg);
  void taskAssignmentTimerCallback(const ros::TimerEvent& e);
  void taskAssignmentMsgCallback(const heterogeneous_manager::ViewpointsTaskConstPtr& msg);
  void visitedTourTimerCallback(const ros::TimerEvent& e);
  void visitedTourMsgCallback(const heterogeneous_manager::VisitedTourConstPtr& msg);

  // Swarm state
  ros::Publisher drone_state_pub_, task_assignment_pub_, visited_tour_pub_;
  ros::Subscriber drone_state_sub_, task_assignment_sub_, visited_tour_sub_;
  ros::Timer drone_state_timer_, task_assignment_timer_, visited_tour_timer_;

public:
  HeterogeneousPlannerFSM(/* args */)
  {
  }
  ~HeterogeneousPlannerFSM()
  {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace hetero_planner

#endif