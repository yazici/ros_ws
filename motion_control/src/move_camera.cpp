#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <cmath>

int main(int argc, char** argv)
{
  ros::init( argc, argv, "motion_control" );
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // create interface for motion planning
  static const std::string PLANNING_GROUP = "camera";
  moveit::planning_interface::MoveGroupInterface move_group( PLANNING_GROUP );
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  ROS_INFO_NAMED( "motion_ctrl", "Reference frame: %s", move_group.getPlanningFrame().c_str() );
  ROS_INFO_NAMED( "motion_ctrl", "End effector link: %s", move_group.getEndEffectorLink().c_str() );
  ros::Rate rate( 0.5 );

  while ( ros::ok() )
  {
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions( joint_model_group, joint_group_positions );
    std::cout << joint_group_positions[3] << std::endl;
    if ( std::abs( joint_group_positions[3] + 1.2994 ) <= 0.01 )
    {
      joint_group_positions[3] = -0.069;  // radians
    } else if ( std::abs( joint_group_positions[3] + 0.069 ) <= 0.01 )
    {
        joint_group_positions[3] = -1.2994;  // radians
    }
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = ( move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    if (success)
    {
      move_group.setMaxVelocityScalingFactor(0.01);
      move_group.setMaxAccelerationScalingFactor(0.01);
      move_group.move();
    }

    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
