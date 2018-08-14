cmake_minimum_required(VERSION 2.8.3)
project(object_localizer)

add_compile_options(-std=c++11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)

find_package( catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  OpenCV
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  image_transport
  object_localizer_msg
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES object_localizer
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  #  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pcl_merger src/pcl_merger.cpp)
target_link_libraries(pcl_merger ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(rough_localizer src/rough_localizer.cpp)
target_link_libraries(rough_localizer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(box_segmenter src/box_segmenter.cpp)
target_link_libraries(box_segmenter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_planner src/scan_planner.cpp)
target_link_libraries(scan_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
