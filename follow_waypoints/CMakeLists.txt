cmake_minimum_required(VERSION 2.8.3)
project(follow_waypoints)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pluginlib
  moveit_core
  moveit_fake_controller_manager
  moveit_ros_planning_interface
  moveit_ros_perception
  interactive_markers
  moveit_visual_tools
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

add_executable(Test src/Test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS Test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines ${catkin_LIBRARIES})

add_executable(simple_marker src/simple_marker.cpp)
target_link_libraries(simple_marker ${catkin_LIBRARIES})

add_executable(basic_controls src/basic_controls.cpp)
target_link_libraries(basic_controls ${catkin_LIBRARIES})

add_executable(menu src/menu.cpp)
target_link_libraries(menu ${catkin_LIBRARIES})

add_executable(MyLineMarker src/MyLineMarker.cpp)
target_link_libraries(MyLineMarker ${catkin_LIBRARIES})
