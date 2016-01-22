cmake_minimum_required(VERSION 2.8.3)
project(grasp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_msgs
  tf
  moveit_ros_planning_interface
 
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(moveB src/moveBaxter.cpp)
target_link_libraries(moveB ${catkin_LIBRARIES})
