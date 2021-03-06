cmake_minimum_required(VERSION 2.8.3)
project(motion_control)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    tf2
    tf2_ros
    std_srvs
    pcl_ros
    pcl_conversions
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS
    common/include
  LIBRARIES
    modbus
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

add_library(modbus common/src/modbus.cpp)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} common/include)
link_directories(${catkin_LIBRARY_DIRS})

add_executable(motion_control src/motion_control.cpp)
target_link_libraries(motion_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(control_node_b src/control_node_b.cpp)
target_link_libraries(control_node_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_camera src/move_camera.cpp)
target_link_libraries(move_camera ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_camera_b src/move_camera_b.cpp)
target_link_libraries(move_camera_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(do_scan src/do_scan.cpp)
target_link_libraries(do_scan ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(do_scan_b src/do_scan_b.cpp)
target_link_libraries(do_scan_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(point_rivet src/point_rivet.cpp)
target_link_libraries(point_rivet ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(point_rivet_b src/point_rivet_b.cpp)
target_link_libraries(point_rivet_b ${catkin_LIBRARIES} ${Boost_LIBRARIES} modbus)

add_executable(pose_test src/pose_test.cpp)
target_link_libraries(pose_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
