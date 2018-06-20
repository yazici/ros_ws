cmake_minimum_required(VERSION 2.8.3)
project(model_matcher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES model_matcher
#  CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(surface_matching_test src/surface_matching_test.cpp)
target_link_libraries(surface_matching_test ${catkin_LIBRARIES})
