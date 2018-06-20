cmake_minimum_required(VERSION 2.8.3)
project(cloud_point_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES follow_waypoints
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud_pub src/point_cloud_pub.cpp)
target_link_libraries(point_cloud_pub ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcd_reader src/pcd_reader.cpp)
target_link_libraries(pcd_reader ${catkin_LIBRARIES} ${Boost_LIBRARIES})
