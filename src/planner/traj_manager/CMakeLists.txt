cmake_minimum_required(VERSION 2.8.3)
project(traj_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
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
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_manager
 CATKIN_DEPENDS plan_env path_searching active_perception traj_utils)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(traj_manager
  src/traj_generator.cpp
  )
target_link_libraries(traj_manager 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
