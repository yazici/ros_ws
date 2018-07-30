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
  message_generation
)

add_message_files(
  FILES
  BBox.msg
  BBox_list.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES opencv_test
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  #  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(pcl_processor src/pcl_processor.cpp)
target_link_libraries(pcl_processor ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
