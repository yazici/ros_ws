#include <cmath>
#include <queue>
#include <iostream>
#include <string>

#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

class Target
{
public:
  int id;
  double x, y, z, roll, pitch, yaw;
  Target ( int id_i, double x_i, double y_i, double z_i, double roll_i, double pitch_i, double yaw_i);
};

Target::Target ( int id_i, double x_i, double y_i, double z_i, double roll_i, double pitch_i, double yaw_i)
{
  id = id_i;
  x = x_i;
  y = y_i;
  z = z_i;
  roll = roll_i;
  pitch = pitch_i;
  yaw = yaw_i;
}

void CfgFileReader ( std::queue< Target >& target_queue )
{
  std::string cfgFileName = ros::package::getPath ( "object_localizer" ) + "/config/point_rivet.cfg";
  std::cout << "***The path of the point_rivet configuration file is: [" << cfgFileName << "]" << std::endl;

  int id;
  double x, y, z, roll, pitch, yaw;
  std::ifstream input ( cfgFileName );
  std::string line;
  while ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> id >> x >> y >> z >> roll >> pitch >> yaw;
    Target target ( id, x, y, z, roll, pitch, yaw );
    target_queue.push ( target );
    std::cout << id << ": [x, y, z, roll, pitch, yaw] = [" << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
  }
  input.close();
}

void do_point_rivet ()
{
  std::queue< Target > target_queue;
  CfgFileReader ( target_queue );
  // create interface for motion planning
  static const std::string PLANNING_GROUP = "rivet_tool";
  moveit::planning_interface::MoveGroupInterface move_group ( PLANNING_GROUP );
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO_NAMED( "point_rivet", "Reference frame: %s", move_group.getPlanningFrame ().c_str () );
  ROS_INFO_NAMED( "point_rivet", "End effector link: %s", move_group.getEndEffectorLink ().c_str () );

  if ( !target_queue.empty () )
  {
    Target target = target_queue.front ();
    target_queue.pop ();
    geometry_msgs::Pose target_pose1;
  	target_pose1.position.x = target.x;
    target_pose1.position.y = target.y;
  	target_pose1.position.z = target.z;
    float rollt  = target.roll;
    float pitcht = target.pitch;
    float yawt   = target.yaw;
    target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
    move_group.setPoseTarget ( target_pose1 );
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO_STREAM ( "Start for planning" );
    bool success = ( move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    ROS_INFO_NAMED ( "point_rivet", "planning for goal pose is %s", success ? "success" : "FAILED" );

    if ( success )
    {
      move_group.setMaxVelocityScalingFactor ( 0.05 );
      move_group.setMaxAccelerationScalingFactor ( 0.05 );
      move_group.move ();
      ros::Duration ( 1.0 ) .sleep ();

      // start scanning the part
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back ( target_pose1 );

      while ( !target_queue.empty () )
      {
        Target target = target_queue.front();
        target_queue.pop();
        geometry_msgs::Pose target_pose2 = target_pose1;
        target_pose2.position.x = target.x;
        target_pose2.position.y = target.y;
      	target_pose2.position.z = target.z;
        rollt  = target.roll;
        pitcht = target.pitch;
        yawt   = target.yaw;
        target_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
        waypoints.push_back ( target_pose2 );
      }

      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group.computeCartesianPath ( waypoints, eef_step, jump_threshold, trajectory );
      ROS_INFO_NAMED ( "point_rivet", "scan plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0 );

      if ( fraction > 0.98 )
      {
        // scale the velocity and acceleration of the trajectory
        const double scale_factor = 0.15;
        int point_size = trajectory.joint_trajectory.points.size();
        for ( int point_idx = 0; point_idx < point_size; point_idx++ )
        {
          trajectory_msgs::JointTrajectoryPoint point_tmp = trajectory.joint_trajectory.points[point_idx];
          int size_tmp = point_tmp.velocities.size();
          for ( int i = 0; i <= size_tmp; i++ )
          {
            float velocity_tmp = point_tmp.velocities[i];
            trajectory.joint_trajectory.points[point_idx].velocities[i] = velocity_tmp * scale_factor;
            float acceleration_tmp = point_tmp.accelerations[i];
            trajectory.joint_trajectory.points[point_idx].accelerations[i] = acceleration_tmp * scale_factor;
          }
          ros::Duration time_from_start_tmp = point_tmp.time_from_start;
          trajectory.joint_trajectory.points[point_idx].time_from_start.fromSec ( time_from_start_tmp.toSec() / scale_factor );
        }

        my_plan.trajectory_ = trajectory;

        move_group.execute ( my_plan );
        ros::Duration ( 5.0 ) .sleep ();
      }
    }

  }
}

bool start_point_rivet ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  do_point_rivet ();
  return true;
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "point_rivet" );
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::ServiceServer start_point_rivet_;
  start_point_rivet_ = nh_.advertiseService ( "start_point_rivet", &start_point_rivet );
  ros::waitForShutdown ();
  return 0;
}
