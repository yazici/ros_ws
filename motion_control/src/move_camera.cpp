#include <cmath>
#include <iostream>

#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

bool is_move = false;
ros::ServiceClient end_generate_scan_plan_;

void move_camera ()
{
  static const std::string PLANNING_GROUP = "camera";
  moveit::planning_interface::MoveGroupInterface move_group ( PLANNING_GROUP );
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
  ROS_INFO_NAMED ( "move_camera", "Reference frame: %s", move_group.getPlanningFrame().c_str() );
  ROS_INFO_NAMED ( "move_camera", "End effector link: %s", move_group.getEndEffectorLink().c_str() );
  ros::Rate rate ( 2 );

  while ( ros::ok () )
  {
    // std::cout << "is_move = " << is_move << std::endl;
    if ( is_move )
    {
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
      std::vector < double > joint_group_positions;
      current_state->copyJointGroupPositions ( joint_model_group, joint_group_positions );
      std::cout << "current angle: " << joint_group_positions[3] << std::endl;
      // if the start robot pose is pose 2
      if ( std::abs ( joint_group_positions[3] + 1.2994 ) <= 0.01 )
      {
        int motion_stage_idx = 0;
        while ( motion_stage_idx != 2 )
        {
          if ( motion_stage_idx  == 0 )
          {
            // set pose 3 in radians
            joint_group_positions [ 3 ] = -0.069;
          } else if ( motion_stage_idx  == 1 )
          {
            // set pose 2 in radians
            joint_group_positions [ 3 ] = -1.2994;
          }
          move_group.setJointValueTarget ( joint_group_positions );
          moveit::planning_interface::MoveGroupInterface::Plan my_plan;
          bool success = ( move_group.plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
          if ( success )
          {
            std::cout << "move to: " << joint_group_positions [ 3 ] << std::endl;
            move_group.setMaxVelocityScalingFactor ( 0.01 );
            move_group.setMaxAccelerationScalingFactor ( 0.01 );
            move_group.move ();
          }
          motion_stage_idx ++;
        }
        // call the end_generate_scan_plan service from node control_node
        if ( motion_stage_idx == 2 )
        {
          std_srvs::Empty msg;
          end_generate_scan_plan_.call ( msg );
          is_move = false;
        }
      }
    }
    rate.sleep ();
  }
}

bool start_move_camera ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  is_move = true;
  return true;
}

bool stop_move_camera ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  is_move = false;
  return true;
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "move_camera" );
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ros::ServiceServer start_move_camera_, stop_move_camera_;
  start_move_camera_ = nh_.advertiseService ( "start_move_camera", &start_move_camera );
  stop_move_camera_ = nh_.advertiseService ( "stop_move_camera", &stop_move_camera );
  end_generate_scan_plan_ = nh_.serviceClient < std_srvs::Empty > ( "end_generate_scan_plan" );
  is_move = false;
  move_camera ();
  ros::waitForShutdown ();
  return 0;
}
