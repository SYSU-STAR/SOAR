#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolyTraj.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <active_perception/perception_utils.h>

#include <gcopter/trajectory.hpp>

#include <traj_manager/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}
using hetero_planner::PerceptionUtils;
using namespace std;

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;
nav_msgs::Odometry odom;
quadrotor_msgs::PositionCommand cmd;

// Info of generated traj
double traj_duration_;
double traj_output_deltaT_ = 0.5;
ros::Time start_time_;
int traj_id_;
int pub_traj_id_;
int drone_id_, drone_num_;
string map_name_, output_folder_;

std::shared_ptr<Trajectory<7>> minco_traj_;
std::shared_ptr<Trajectory<7>> minco_yaw_traj_;
std::shared_ptr<Trajectory<7>> minco_pitch_traj_;

shared_ptr<PerceptionUtils> percep_utils_;

// Info of replan
double replan_time_;
bool receive_traj_ = false;

// Executed traj, commanded and real ones
vector<Eigen::Vector3d> traj_cmd_, traj_real_;

// Data for benchmark comparison
ros::Time start_time, end_time, last_time;
double energy;

// output traj
ofstream traj_file;
double last_record_time = -traj_output_deltaT_;

double calcPathLength(const vector<Eigen::Vector3d>& path)
{
  if (path.empty())
    return 0;
  double len = 0.0;
  for (int i = 0; i < (int)path.size() - 1; ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

void displayTrajWithColor(
    vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.06;
  mk.scale.y = 0.06;
  mk.scale.z = 0.06;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  cmd_vis_pub.publish(mk);

  if (list1.size() == 0)
    return;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  cmd_vis_pub.publish(mk);
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
    const Eigen::Vector4d& color)
{
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void replanCallback(std_msgs::Empty msg)
{
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.3;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

void odomCallbck(const nav_msgs::Odometry& msg)
{
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O")
    return;
  odom = msg;
  traj_real_.push_back(Eigen::Vector3d(
      odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000)
    traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void mincoTrajCallback(quadrotor_msgs::PolyTrajPtr msg)
{
  if (msg->order != 7) {
    ROS_ERROR("[traj_server] Only support trajectory order equals 7 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  int yaw_piece_nums = msg->yaw_duration.size();
  int pitch_piece_nums = msg->pitch_duration.size();
  std::vector<double> durs(piece_nums);
  std::vector<double> yaw_durs(yaw_piece_nums);
  std::vector<double> pitch_durs(pitch_piece_nums);
  vector<Eigen::Matrix<double, 3, 8>> cMats(piece_nums);
  vector<Eigen::Matrix<double, 3, 8>> yaw_cMats(yaw_piece_nums);
  vector<Eigen::Matrix<double, 3, 8>> pitch_cMats(pitch_piece_nums);

  for (int i = 0; i < piece_nums; ++i) {
    int i8 = i * 8;
    cMats[i].row(0) << msg->coef_x[i8 + 7], msg->coef_x[i8 + 6], msg->coef_x[i8 + 5],
        msg->coef_x[i8 + 4], msg->coef_x[i8 + 3], msg->coef_x[i8 + 2], msg->coef_x[i8 + 1],
        msg->coef_x[i8 + 0];
    cMats[i].row(1) << msg->coef_y[i8 + 7], msg->coef_y[i8 + 6], msg->coef_y[i8 + 5],
        msg->coef_y[i8 + 4], msg->coef_y[i8 + 3], msg->coef_y[i8 + 2], msg->coef_y[i8 + 1],
        msg->coef_y[i8 + 0];
    cMats[i].row(2) << msg->coef_z[i8 + 7], msg->coef_z[i8 + 6], msg->coef_z[i8 + 5],
        msg->coef_z[i8 + 4], msg->coef_z[i8 + 3], msg->coef_z[i8 + 2], msg->coef_z[i8 + 1],
        msg->coef_z[i8 + 0];
    durs[i] = msg->duration[i];
  }
  for (int i = 0; i < yaw_piece_nums; ++i) {
    int i8 = i * 8;
    yaw_cMats[i].row(0) << msg->yaw_coef_x[i8 + 7], msg->yaw_coef_x[i8 + 6],
        msg->yaw_coef_x[i8 + 5], msg->yaw_coef_x[i8 + 4], msg->yaw_coef_x[i8 + 3],
        msg->yaw_coef_x[i8 + 2], msg->yaw_coef_x[i8 + 1], msg->yaw_coef_x[i8 + 0];
    yaw_cMats[i].row(1) << msg->yaw_coef_y[i8 + 7], msg->yaw_coef_y[i8 + 6],
        msg->yaw_coef_y[i8 + 5], msg->yaw_coef_y[i8 + 4], msg->yaw_coef_y[i8 + 3],
        msg->yaw_coef_y[i8 + 2], msg->yaw_coef_y[i8 + 1], msg->yaw_coef_y[i8 + 0];
    yaw_cMats[i].row(2) << msg->yaw_coef_z[i8 + 7], msg->yaw_coef_z[i8 + 6],
        msg->yaw_coef_z[i8 + 5], msg->yaw_coef_z[i8 + 4], msg->yaw_coef_z[i8 + 3],
        msg->yaw_coef_z[i8 + 2], msg->yaw_coef_z[i8 + 1], msg->yaw_coef_z[i8 + 0];
    yaw_durs[i] = msg->yaw_duration[i];
  }
  for (int i = 0; i < pitch_piece_nums; ++i) {
    int i8 = i * 8;
    pitch_cMats[i].row(0) << msg->pitch_coef_x[i8 + 7], msg->pitch_coef_x[i8 + 6],
        msg->pitch_coef_x[i8 + 5], msg->pitch_coef_x[i8 + 4], msg->pitch_coef_x[i8 + 3],
        msg->pitch_coef_x[i8 + 2], msg->pitch_coef_x[i8 + 1], msg->pitch_coef_x[i8 + 0];
    pitch_cMats[i].row(1) << msg->pitch_coef_y[i8 + 7], msg->pitch_coef_y[i8 + 6],
        msg->pitch_coef_y[i8 + 5], msg->pitch_coef_y[i8 + 4], msg->pitch_coef_y[i8 + 3],
        msg->pitch_coef_y[i8 + 2], msg->pitch_coef_y[i8 + 1], msg->pitch_coef_y[i8 + 0];
    pitch_cMats[i].row(2) << msg->pitch_coef_z[i8 + 7], msg->pitch_coef_z[i8 + 6],
        msg->pitch_coef_z[i8 + 5], msg->pitch_coef_z[i8 + 4], msg->pitch_coef_z[i8 + 3],
        msg->pitch_coef_z[i8 + 2], msg->pitch_coef_z[i8 + 1], msg->pitch_coef_z[i8 + 0];
    pitch_durs[i] = msg->pitch_duration[i];
  }

  minco_traj_.reset(new Trajectory<7>(durs, cMats));
  minco_yaw_traj_.reset(new Trajectory<7>(yaw_durs, yaw_cMats));
  minco_pitch_traj_.reset(new Trajectory<7>(pitch_durs, pitch_cMats));

  start_time_ = msg->start_time;
  traj_duration_ = minco_traj_->getTotalDuration();

  traj_id_ = msg->traj_id;

  receive_traj_ = true;

  if (start_time.isZero()) {
    ROS_WARN("start flight");
    start_time = ros::Time::now();

    string TrajFile = output_folder_ + "/" + map_name_ + to_string(drone_id_) + ".txt";
    traj_file.open(TrajFile);
  }
}

void visCallback(const ros::TimerEvent& e)
{
  // Draw the executed traj (desired state)
  Eigen::Vector4d col;
  if (drone_id_ == 0)
    col = Eigen::Vector4d(0, 0, 0, 1);
  else if (drone_id_ == 1)
    col = Eigen::Vector4d(1, 0, 0, 1);
  else if (drone_id_ == 2)
    col = Eigen::Vector4d(0, 1, 0, 1);
  else if (drone_id_ == 3)
    col = Eigen::Vector4d(0, 0, 1, 1);
  else
    col = Eigen::Vector4d(1, 1, 0, 1);
  // double r = 1.0 - static_cast<double>(drone_id_) / static_cast<double>(drone_num_); // Vary
  // red component from 0 to 1 double g = 0.5; // Constant green component double b =
  // static_cast<double>(drone_id_) / static_cast<double>(drone_num_);       // Constant blue
  // component
  if (drone_id_ == 0)
    displayTrajWithColor(traj_cmd_, 0.15, col, pub_traj_id_);
  else
    displayTrajWithColor(traj_cmd_, 0.15, col, pub_traj_id_);
}

void outputTraj(double timestamp, Eigen::Vector3d pos, double pitch, double yaw)
{
  string traj_str_;
  traj_str_ = "TIMESTAMP: " + to_string(timestamp) + ", X: " + to_string(pos(0)) +
              ", Y: " + to_string(pos(1)) + ", Z: " + to_string(pos(2)) +
              ", PITCH: " + to_string(pitch) + ", YAW: " + to_string(yaw) + "\n";
  traj_file << traj_str_;
}

void cmdCallback(const ros::TimerEvent& e)
{
  // No publishing before receive traj data
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos, vel, acc, jer;
  double yaw, yawdot;
  double pitch, pitchdot;
  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    // Current time within range of planned traj
    pos = minco_traj_->getPos(t_cur);
    vel = minco_traj_->getVel(t_cur);
    acc = minco_traj_->getAcc(t_cur);
    jer = minco_traj_->getJer(t_cur);
    Eigen::Vector3d ap = minco_yaw_traj_->getPos(t_cur);
    yaw = atan2(ap(1), ap(0) + 1e-6);
    yawdot = minco_yaw_traj_->getVel(t_cur).norm();
    Eigen::Vector3d pitch_ap = minco_pitch_traj_->getPos(t_cur);
    pitch = atan2(pitch_ap(1), pitch_ap(0) + 1e-6);

    double len = calcPathLength(traj_cmd_);
    if (end_time.toSec() > 1e-3) {
      double flight_t = (end_time - start_time).toSec();
      ROS_WARN_THROTTLE(2.0,
          "[Traj_server] drone_%d flight time: %lf, path length: %lf, mean vel: %lf", drone_id_,
          flight_t, len, len / flight_t);
    }

    double t_from_start = (time_now - start_time).toSec();
    static Eigen::VectorXd last_pose1(5);
    Eigen::VectorXd cur_pose(5);
    cur_pose << pos(0), pos(1), pos(2), pitch, yaw;
    if (t_from_start - last_record_time >= traj_output_deltaT_) {
      outputTraj(t_from_start, pos, pitch, yaw);
      last_record_time = t_from_start;
      last_pose1 = cur_pose;
    }
  }
  else if (t_cur >= traj_duration_) {
    // Current time exceed range of planned traj
    // keep publishing the final position and yaw
    pos = minco_traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();
    Eigen::Vector3d ap = minco_yaw_traj_->getPos(traj_duration_);
    yaw = atan2(ap(1), ap(0) + 1e-6);
    yawdot = 0.0;
    Eigen::Vector3d pitch_ap = minco_pitch_traj_->getPos(traj_duration_);
    pitch = atan2(pitch_ap(1), pitch_ap(0) + 1e-6);

    // Report info of the whole flight
    double len = calcPathLength(traj_cmd_);
    double flight_t = (end_time - start_time).toSec();
    ROS_WARN_THROTTLE(2.0,
        "[Traj_server] drone_%d flight time: %lf, path length: %lf, mean vel: %lf", drone_id_,
        flight_t, len, len / flight_t);

    double t_from_start = (time_now - start_time).toSec();
    static Eigen::VectorXd last_pose(5);
    Eigen::VectorXd cur_pose(5);
    cur_pose << pos(0), pos(1), pos(2), pitch, yaw;
    if (t_from_start - last_record_time >= traj_output_deltaT_ &&
        (last_pose - cur_pose).norm() > 0.001) {
      outputTraj(t_from_start, pos, pitch, yaw);
      last_record_time = t_from_start;
      last_pose = cur_pose;
    }
  }
  else {
    cout << "[Traj server]: invalid time." << endl;
  }
  cmd.header.stamp = time_now;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  if (drone_id_ == 0) {
    cmd.yaw = yaw;
    cmd.yaw_dot = yawdot;
  }
  else {
    cmd.yaw = 0.0;
    cmd.yaw_dot = 0.0;
  }
  pos_cmd_pub.publish(cmd);

  // Draw cmd
  // Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  // drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));
  percep_utils_->setPose_PY(pos, pitch, yaw);
  vector<Eigen::Vector3d> l1, l2;
  percep_utils_->getFOV_PY(l1, l2);
  drawFOV(l1, l2);

  // Record info of the executed traj
  if (traj_cmd_.size() == 0) {
    // Add the first position
    traj_cmd_.push_back(pos);
  }
  else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    // Add new different commanded position
    traj_cmd_.push_back(pos);
    double dt = (time_now - last_time).toSec();
    energy += jer.squaredNorm() * dt;
    end_time = ros::Time::now();
  }
  last_time = time_now;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("/planning/minco_traj", 10, mincoTrajCallback);
  ros::Subscriber replan_sub = node.subscribe("/planning/replan", 10, replanCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  nh.param("map_name", map_name_, string("null"));
  nh.param("output_folder", output_folder_, string("null"));
  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);
  nh.param("fsm/replan_time", replan_time_, 0.1);
  nh.param("drone_id", drone_id_, -1);
  nh.param("drone_num", drone_num_, -1);

  Eigen::Vector3d init_pos;
  nh.param("traj_server/init_x", init_pos[0], 0.0);
  nh.param("traj_server/init_y", init_pos[1], 0.0);
  nh.param("traj_server/init_z", init_pos[2], 0.0);

  ROS_WARN("[Traj server]: init...");
  ros::Duration(1.0).sleep();

  // Control parameter
  cmd.kx = { 5.7, 5.7, 6.2 };
  cmd.kv = { 3.4, 3.4, 4.0 };

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

  if (drone_id_ == 0)
    percep_utils_.reset(new PerceptionUtils(nh, 0));
  else
    percep_utils_.reset(new PerceptionUtils(nh, 1));

  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}
