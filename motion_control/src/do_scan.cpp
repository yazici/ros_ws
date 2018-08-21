#include <cmath>
#include <vector>

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_scan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // create interface for motion planning
  static const std::string PLANNING_GROUP = "me_2900";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  ROS_INFO_NAMED("do_scan", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("do_scan", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  ROS_INFO_STREAM ( "Start for planning" );
  geometry_msgs::Pose target_pose1;

  while ( ros::ok() )
  {
    // [***] Rotation around x is [143.606] degrees
    // [***] Scan central point is [x, y, z] = [0.240043, -1.54288, 1.66355]
    // [***] Scan start point is [x, y, z] = [0.240043, -1.59035, 1.59915]
    // [***] Scan end point is [x, y, z] = [0.240043, -1.49542, 1.72795]
    float rotation_deg = 143.606;
    float start_point[3] {0.240043, -1.59035, 1.59915};
    float end_point[3] {0.240043, -1.49542, 1.72795};

  	target_pose1.position.x = start_point[0];
    target_pose1.position.y = start_point[1];
  	target_pose1.position.z = start_point[2];
    float rollt = rotation_deg * M_PI / 180.0;
    float pitcht = 0;
    float yawt = 0;
    target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );

    // move to the start point
    move_group.setPoseTarget ( target_pose1 );
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = ( move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    ROS_INFO_NAMED( "do_scan", "planning for goal pose is %s", success ? "success" : "FAILED" );

    if ( success )
    {
      move_group.setMaxVelocityScalingFactor ( 0.01 );
      move_group.setMaxAccelerationScalingFactor ( 0.01 );
      move_group.move();

      // start scanning the part
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back ( target_pose1 );

      geometry_msgs::Pose target_pose2 = target_pose1;
      target_pose2.position.x = end_point[0];
      target_pose2.position.y = end_point[1];
      target_pose2.position.z = end_point[2];
      rollt = rotation_deg * M_PI / 180.0;
      pitcht = 0;
      yawt = 0;
      target_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
      waypoints.push_back ( target_pose2 );

      waypoints.push_back ( target_pose1 );

      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group.computeCartesianPath ( waypoints, eef_step, jump_threshold, trajectory );
      ROS_INFO_NAMED ( "do_scan", "scan plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0 );

      if ( fraction > 0.98 )
      {
        // scale the velocity and acceleration of the trajectory
        const double scale_factor = 0.02;
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

  ros::shutdown();
  return 0;
}
