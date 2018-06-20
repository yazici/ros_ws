// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' " << " / control '" << feedback->control_name << "'";
  ROS_INFO_STREAM( s.str() << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/world";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "Simple 1-DOF Control";

  // create a red sphere marker at position (x, y, z) = (0, 0.5, 0.5);
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::SPHERE;
	box_marker.pose.position.x = 0;
	box_marker.pose.position.y = 0.5;
	box_marker.pose.position.z = 0.5;
  box_marker.scale.x = 0.4;
  box_marker.scale.y = 0.4;
  box_marker.scale.z = 0.4;
  box_marker.color.r = 1.0;
  box_marker.color.g = 0.0;
  box_marker.color.b = 0.0;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
	box_control.name = "sphere_control";
  box_control.always_visible = true;
	box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  //visualization_msgs::InteractiveMarkerControl rotate_control;
  //rotate_control.name = "MOVE_PLANE"; // ?
  //rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE; // ?
  //int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
