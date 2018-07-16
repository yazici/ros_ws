cmake_minimum_required(VERSION 2.8.3)
project(phasespace_msgs)

##### Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)


################################################################################
## ROS messages, services and actions ##########################################
################################################################################

##### Generate messages in the 'msg' folder

add_message_files(
    DIRECTORY msg
    FILES
    Button.msg
    ButtonsStamped.msg
    Marker.msg
    MarkersStamped.msg
    Peak.msg
    PeaksStamped.msg
    Rigid.msg
    RigidsStamped.msg
)

##### Generate added messages and services with any dependencies listed here

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)


################################################################################
## catkin specific configuration ###############################################
################################################################################

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)


################################################################################
## Qt Creator specific configuration ###########################################
################################################################################

##### Workaround allowing Qt Creator to access/display additional files

set(OTHER_FILES
    msg/Button.msg
    msg/ButtonsStamped.msg
    msg/Marker.msg
    msg/MarkersStamped.msg
    msg/Peak.msg
    msg/PeaksStamped.msg
    msg/Rigid.msg
    msg/RigidsStamped.msg
)

add_custom_target(
    qtcreator_phasespace_msgs
    SOURCES ${OTHER_FILES}
)
