cmake_minimum_required(VERSION 2.8.3)
project(heterogeneous_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_utils
  active_perception
  lkh_mtsp_solver
  cv_bridge
  message_generation
  traj_manager
)

add_message_files(
  FILES
  DroneState.msg
  ViewpointsTask.msg
  VisitedTour.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES heterogeneous_manager
 CATKIN_DEPENDS plan_env path_searching active_perception traj_utils lkh_mtsp_solver message_runtime traj_manager
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(heterogeneous_planner_node
  src/heterogeneous_planner_node.cpp 
  src/heterogeneous_planner_fsm.cpp
  src/heterogeneous_planner_manager.cpp
  src/coverage_manager.cpp
  include/ikd-Tree/ikd_Tree.cpp
  )
target_link_libraries(heterogeneous_planner_node 
  ${catkin_LIBRARIES}
  )
