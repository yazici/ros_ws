#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <std_srvs/Empty.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

static const std::string PLANNING_GROUP = "camera";

class ControlNode {

  ros::NodeHandle nh_;
  tf2_ros::StaticTransformBroadcaster br;
  ros::ServiceClient add_aircraft_frame_, remove_aircraft_frame_;
  ros::ServiceServer end_rough_localization_, end_profile_scan_;

  boost::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group;
  boost::shared_ptr< moveit::planning_interface::PlanningSceneInterface > planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group;

public:

  ControlNode ()
  {
    add_aircraft_frame_ = nh_.serviceClient < std_srvs::Empty > ( "add_aircraft_frame" );
    remove_aircraft_frame_ = nh_.serviceClient < std_srvs::Empty > ( "remove_aircraft_frame" );
    end_rough_localization_ = nh_.advertiseService ( "end_rough_localization", &ControlNode::end_rough_localization, this );
    end_profile_scan_ = nh_.advertiseService ( "end_profile_scan", &ControlNode::end_profile_scan, this );

    move_group.reset ( new moveit::planning_interface::MoveGroupInterface ( PLANNING_GROUP ) );
    planning_scene_interface.reset ( new moveit::planning_interface::PlanningSceneInterface () );
    joint_model_group = move_group->getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
    ROS_INFO_NAMED ( "control_node", "Reference frame: %s", move_group->getPlanningFrame().c_str() );
    ROS_INFO_NAMED ( "control_node", "End effector link: %s", move_group->getEndEffectorLink().c_str() );
  }

  ~ControlNode ()
  {}

  void start_rough_localization ()
  {
    std_srvs::Empty msg;
    if ( add_aircraft_frame_.call ( msg ) )
    {
      std::cout << "aircraft frame has been added" << std::endl;
      std::map < std::string, double > value_map = move_group->getNamedTargetValues ( "pose2" );
      // std::cout << move_group->getNamedTargetValues ( "pose2" ) << std::endl;
      // for ( const auto& value_pair : value_map )
      // {
      //   std::cout << "<" << value_pair.first << "> = <" << value_pair.second << ">\n";
      // }
      move_group->setJointValueTarget ( value_map );
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = ( move_group->plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
      if ( success )
      {
        std::cout << "reset robot pose to pose 2" << std::endl;
        move_group->setMaxVelocityScalingFactor ( 0.1 );
        move_group->setMaxAccelerationScalingFactor ( 0.1 );
        move_group->move ();
      }
    }
  }

  bool end_rough_localization ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    /*
    ** do something
    */
    return true;
  }

  bool end_profile_scan ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    /*
    ** do something
    */
    return true;
  }

};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "control_node" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ControlNode control_node;
  control_node.start_rough_localization ();
  ros::waitForShutdown ();
  return 0;
}
