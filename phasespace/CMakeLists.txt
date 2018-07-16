cmake_minimum_required(VERSION 2.8.3)
project(phasespace)

##### Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    phasespace_msgs
    roscpp
    tf2_ros
    visualization_msgs
)


################################################################################
## ROS messages, services and actions ##########################################
################################################################################

##### Generate messages in the 'msg' folder

#add_message_files(
#    DIRECTORY msg
#    FILES
#)

##### Generate services in the 'srv' folder

#add_service_files(
#    DIRECTORY srv
#    FILES
#)

##### Generate added messages and services with any dependencies listed here

#generate_messages(
#    DEPENDENCIES
#)


################################################################################
## ROS dynamic reconfigure parameters ##########################################
################################################################################

##### Generate dynamic reconfigure parameters in the 'cfg' folder

generate_dynamic_reconfigure_options(
    cfg/ParametersPsOwl.cfg
)


################################################################################
## catkin specific configuration ###############################################
################################################################################

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS dynamic_reconfigure phasespace_msgs roscpp tf2_ros visualization_msgs
#    DEPENDS yaml-cpp
)


################################################################################
## Build configuration #########################################################
################################################################################

##### Enable C++14 compiler support

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

##### Specify additional locations of header files

include_directories(
    sdk/include
    ${catkin_INCLUDE_DIRS}
)

##### Specify additional locations of libraries

link_directories(
    sdk/lib
)

##### Declare a C++ executable
##### Add cmake target dependencies of the executable
##### Specify libraries to link a library or executable target against

add_executable( ps_owl src/ps_owl.cpp )
add_dependencies( ps_owl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg )
target_link_libraries( ps_owl ${catkin_LIBRARIES} owlsock yaml-cpp )

add_executable( tf_info src/tf_info.cpp )
add_dependencies( tf_info ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( tf_info ${catkin_LIBRARIES} )

add_executable( vis_info src/vis_info.cpp )
add_dependencies( vis_info ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( vis_info ${catkin_LIBRARIES} )

add_executable( ps_align src/ps_align.cpp )
add_dependencies( ps_align ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ps_align ${catkin_LIBRARIES} )

add_executable( ps_test src/ps_test.cpp )
add_dependencies( ps_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ps_test ${catkin_LIBRARIES} )


################################################################################
## Qt Creator specific configuration ###########################################
################################################################################

##### Workaround allowing Qt Creator to access/display additional files

set(OTHER_FILES
    cfg/ParametersPsOwl.cfg
    config/trackers.yaml
    include/phasespace/helpers.h
    launch/align_system.launch
    launch/stream_data.launch
    launch/test_system.launch
    sdk/include/owl.hpp
    support/calibration/miniwand.json
    README.md
)

add_custom_target(
    qtcreator_phasespace
    SOURCES ${OTHER_FILES}
)
