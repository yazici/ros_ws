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
  ros::init(argc, argv, "motion_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // create interface for motion planning
  static const std::string PLANNING_GROUP = "camera";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  ROS_INFO_NAMED("motion_ctrl", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("motion_ctrl", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // listen to the change of table coordinate
  tf::TransformListener listener;
  ros::Rate rate(1.0);

  float x_o = 10000.0;
  float y_o = 10000.0;
  float angle_o = 10000.0;

  while ( ros::ok() )
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("floor", "table", ros::Time(0), transform);
    }
    catch ( tf::TransformException ex )
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    float x_n = transform.getOrigin().x();
    float y_n = transform.getOrigin().y();
    tf::Vector3 axis_n = transform.getRotation().getAxis();
    float angle_n = transform.getRotation().getAngle();

    ROS_INFO_STREAM ( "x_n: " << x_n << " y_n: " << y_n << " axis_n: [" << axis_n.getX() << ", " << axis_n.getY() << ", " << axis_n.getZ() << "], angle_n = " << angle_n );
    ///*
    if ( std::abs( x_n - x_o ) > 0.03 || std::abs( y_n - y_o ) > 0.03 || std::abs( ( angle_n - angle_o ) / 3.1415 * 180.0 ) > 0.5 )
    {
      ROS_INFO_STREAM ( "start for planning" );
      geometry_msgs::Pose target_pose1;
      tf2::Quaternion q;
      q.setRPY( 0.0, 0.0, 0.0 );

      // target_pose1.orientation.x = q.x();
      // target_pose1.orientation.y = q.y();
      // target_pose1.orientation.z = q.z();
      // target_pose1.orientation.w = q.w();
      // pose 2
      // position:
      // x: -0.2705763764
      // y: -0.514447887003
      // z: 1.37472028611
      // orientation:
      // x: 0.335698187305
      // y: 0.329972059377
      // z: -0.618479609793
      // w: 0.629212316584
      // pose x
      // position:
      // x: -0.270567446638
      // y: -0.633357894959
      // z: 1.44762397584
      // orientation:
      // x: 0.429557969934
      // y: 0.424991905019
      // z: -0.557796409466
      // w: 0.568968361794
      target_pose1.position.x = -0.270567446638;
      target_pose1.position.y = -0.633357894959;
      target_pose1.position.z = 1.44762397584;
      target_pose1.orientation.x = 0.429557969934;
      target_pose1.orientation.y = 0.424991905019;
      target_pose1.orientation.z = -0.557796409466;
      target_pose1.orientation.w = 0.568968361794;

      move_group.setPoseTarget(target_pose1);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("motion_ctrl", "planning for goal pose is %s", success ? "success" : "FAILED");
      if (success)
      {
        move_group.setMaxVelocityScalingFactor(0.1);
        move_group.move();
      }
    }
    //*/
    x_o = x_n;
    y_o = y_n;
    angle_o = angle_n;
    rate.sleep();
  }

  // Planning to a joint-space goal
  /*
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "success" : "FAILED");
  move_group.move();
  */

  /*
  // Cartesian Paths
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  */

  ros::shutdown();
  return 0;
}
