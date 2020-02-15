cmake_minimum_required(VERSION 2.8.3)
project(scan_matching)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
)
find_package(Eigen3 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES emu_imu_scan_matching
   CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(student_scan_matcher src/scan_match.cpp src/transform.cpp src/correspond.cpp src/visualization.cpp)
target_link_libraries(student_scan_matcher ${catkin_LIBRARIES})
#add_dependencies(scan_matcher emu_imu_scan_matching_generate_messages_cpp)
