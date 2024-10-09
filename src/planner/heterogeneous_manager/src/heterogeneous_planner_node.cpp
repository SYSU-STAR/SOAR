#include <ros/ros.h>
#include <heterogeneous_manager/heterogeneous_planner_fsm.h>
#include <heterogeneous_manager/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace hetero_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hetero_planner_node");
  ros::NodeHandle nh("~");

  HeterogeneousPlannerFSM hetero_fsm;
  hetero_fsm.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
