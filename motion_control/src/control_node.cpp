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
  ros::ServiceClient add_aircraft_frame_, remove_aircraft_frame_, start_pcl_merger_, stop_pcl_merger_,
                     start_rough_localizer_, stop_rough_localizer_, start_box_segmenter_, stop_box_segmenter_,
                     start_scan_planner_, stop_scan_planner_, start_move_camera_, stop_move_camera_;
  ros::ServiceClient start_do_scan_, start_point_cloud_writer_, start_rivet_localizer_, start_point_rivet_;
  ros::ServiceServer end_generate_scan_plan_, start_profile_scan_, stop_profile_scan_;

  boost::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group;
  boost::shared_ptr< moveit::planning_interface::PlanningSceneInterface > planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group;

public:

  ControlNode ()
  {
    //  add_aircraft_frame and remove_aircraft_frame
    add_aircraft_frame_ = nh_.serviceClient < std_srvs::Empty > ( "add_aircraft_frame" );
    remove_aircraft_frame_ = nh_.serviceClient < std_srvs::Empty > ( "remove_aircraft_frame" );
    // pcl_merger, rough_localizer, box_segmenter, scan_planner and move_camera
    start_pcl_merger_ = nh_.serviceClient < std_srvs::Empty > ( "start_pcl_merger" );
    stop_pcl_merger_ = nh_.serviceClient < std_srvs::Empty > ( "stop_pcl_merger" );
    start_rough_localizer_ = nh_.serviceClient < std_srvs::Empty > ( "start_rough_localizer" );
    stop_rough_localizer_ = nh_.serviceClient < std_srvs::Empty > ( "stop_rough_localizer" );
    start_box_segmenter_ = nh_.serviceClient < std_srvs::Empty > ( "start_box_segmenter" );
    stop_box_segmenter_ = nh_.serviceClient < std_srvs::Empty > ( "stop_box_segmenter" );
    start_scan_planner_ = nh_.serviceClient < std_srvs::Empty > ( "start_scan_planner" );
    stop_scan_planner_ = nh_.serviceClient < std_srvs::Empty > ( "stop_scan_planner" );
    start_move_camera_ = nh_.serviceClient < std_srvs::Empty > ( "start_move_camera" );
    stop_move_camera_ = nh_.serviceClient < std_srvs::Empty > ( "stop_move_camera" );
    start_do_scan_ = nh_.serviceClient < std_srvs::Empty > ( "start_do_scan" );
    start_point_cloud_writer_ = nh_.serviceClient < std_srvs::Empty > ( "start_point_cloud_writer" );
    start_rivet_localizer_ = nh_.serviceClient < std_srvs::Empty > ( "start_rivet_localizer" );
    start_point_rivet_ = nh_.serviceClient < std_srvs::Empty > ( "start_point_rivet" );

    end_generate_scan_plan_ = nh_.advertiseService ( "end_generate_scan_plan", &ControlNode::end_generate_scan_plan, this );
    start_profile_scan_ = nh_.advertiseService ( "start_profile_scan", &ControlNode::start_profile_scan, this );
    stop_profile_scan_ = nh_.advertiseService ( "stop_profile_scan", &ControlNode::stop_profile_scan, this );

    move_group.reset ( new moveit::planning_interface::MoveGroupInterface ( PLANNING_GROUP ) );
    planning_scene_interface.reset ( new moveit::planning_interface::PlanningSceneInterface () );
    joint_model_group = move_group->getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
    ROS_INFO_NAMED ( "control_node", "Reference frame: %s", move_group->getPlanningFrame().c_str() );
    ROS_INFO_NAMED ( "control_node", "End effector link: %s", move_group->getEndEffectorLink().c_str() );
  }

  ~ControlNode ()
  {}

  bool set_pose2 ()
  {
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
      std::cout << "reset the robot pose to pose 2" << std::endl;
      move_group->setMaxVelocityScalingFactor ( 0.1 );
      move_group->setMaxAccelerationScalingFactor ( 0.1 );
      move_group->move ();
    }
    return success;
  }

  void generate_scan_plan ()
  {
    // step 1, add aircraft_frame to the planning scene.
    std_srvs::Empty msg;
    if ( add_aircraft_frame_.call ( msg ) )
    {
      std::cout << "Aircraft frame has been added" << std::endl;

      // step 2, set the robot pose to pose 2.
      if ( set_pose2 () )
      {
        // step 3, start service pcl_merger, rough_localizer, box_segmenter, and scan_planner
        if ( start_pcl_merger_.call ( msg ) && start_rough_localizer_.call ( msg )
             && start_box_segmenter_.call ( msg ) )
        {
          // step 4, start service move_camera
          start_move_camera_.call ( msg );
        }
      }
    }
  }

  bool end_generate_scan_plan ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    // step 1, stop services pcl_merger, rough_localizer, box_segmenter, and scan_planner
    std_srvs::Empty msg;
    if ( stop_pcl_merger_.call ( msg ) && stop_rough_localizer_.call ( msg ) && stop_box_segmenter_.call ( msg ) )
    {
      // step 2, generate scanning plans and write it to the configuration file [do_scan]
      start_scan_planner_.call ( msg );
    }
    return true;
  }

  bool start_profile_scan ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    std_srvs::Empty msg;
    start_do_scan_.call ( msg );
    return true;
  }

  bool stop_profile_scan ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    std_srvs::Empty msg;
    start_point_cloud_writer_.call ( msg );
    // call service rivet_localizer
    start_rivet_localizer_.call ( msg );
    // call service point_rivet
    start_point_rivet_.call ( msg );
    return true;
  }

};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "control_node" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ControlNode control_node;
  control_node.generate_scan_plan ();
  ros::waitForShutdown ();
  return 0;
}
