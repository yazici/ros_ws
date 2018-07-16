cmake_minimum_required(VERSION 2.8.3)
project(tams_phasespace)

# FNH: only Ubuntu 15.04++, not on 12.04
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # eigen
  roscpp
  urdf
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  phasespace_msgs
  tf
  tf_conversions
  tf2_bullet
  dynamic_reconfigure
  octomap_ros
  octomap_msgs
  interactive_markers
)


#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/WorkspaceParameters.cfg
  #...
)

# find_package(orocos_kdl REQUIRED)
# find_package(Boost REQUIRED COMPONENTS thread program_options)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   PhaseSpaceMarkers.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    sensor_msgs
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include # ok.
 #  LIBRARIES pa10lib # but this is NOT needed, apparantly.
 CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs phasespace_msgs message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  /usr/local/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


link_directories(
  lib
  /usr/local/lib
  ${catkin_LIBRARIES}
  # /opt/ReflexxesTypeII/Linux/x64/release/lib/static
)



## Declare a cpp library
# add_library(pa10lib STATIC
#   src/pa10_utils.cpp
#   src/pa10_traj.cpp
# )

## Declare a cpp executable

add_executable(calibration_visualizer src/calibration_visualizer.cpp)
target_link_libraries( calibration_visualizer ${catkin_LIBRARIES} pthread )
add_dependencies(calibration_visualizer
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


# make sure configure headers are built before any node using them
add_executable(workspace_visualizer src/workspace_visualizer.cpp)
target_link_libraries( workspace_visualizer ${catkin_LIBRARIES} pthread )
add_dependencies(workspace_visualizer
  ${PRJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


add_executable(stylus_node src/stylus_node.cpp)
target_link_libraries( stylus_node ${catkin_LIBRARIES} pthread )
add_dependencies(stylus_node
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(LineIteractiveMarker src/LineIteractiveMarker.cpp)
target_link_libraries(LineIteractiveMarker ${catkin_LIBRARIES})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(find_user_exekutor_node find_user_exekutor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(tams_phasespace ${catkin_LIBRARIES} libowl pthread)
