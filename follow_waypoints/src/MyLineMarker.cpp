#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <cmath>
#include <string>

void make_line(ros::Publisher& marker_pub, int counter) {
	// define the point and line_strip marker
	visualization_msgs::Marker points, line_strip;
	points.header.frame_id = line_strip.header.frame_id = "/world";
	points.header.stamp = line_strip.header.stamp = ros::Time::now();
	points.ns = line_strip.ns = "points_and_lines";
	points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
	
	points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

	// POINTS markers use x and y scale for width/height respectively
 	points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.3;

	// Points are green
  points.color.r = 1.0;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

	// dynamic change points when necessary
	for (uint32_t i = 0; i < 100/5*counter; ++i)
  {
    float y = 5 * sin(i / 100.0f * 2 * M_PI);
    float z = 5 * cos(i / 100.0f * 2 * M_PI);

    geometry_msgs::Point p;
    p.x = (int32_t)i - 50;
    p.y = y;
    p.z = z;

    points.points.push_back(p);
    line_strip.points.push_back(p);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);

}

// define the server for interactive markers.
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
// define the menuhandler.
interactive_markers::MenuHandler menu_handler;

void updateFrame (tf::Transform& t) {
	static tf::TransformBroadcaster br;
  ros::Time time = ros::Time::now();
  br.sendTransform(tf::StampedTransform(t, time, "world", "moving_frame"));
}

///*


static int marker_counter = 1;

void make6DofMarker(unsigned int, std::string);

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' " << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		{
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
			if (feedback->menu_entry_id == 2)
			{
				make6DofMarker(visualization_msgs::InteractiveMarkerControl::MOVE_3D, "6DoF" + std::to_string(marker_counter));
				marker_counter++;
			}
      break;
		}

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		{
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
			tf::Transform t;
			t.setOrigin(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
  		t.setRotation(tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w));
			updateFrame(t);
      break;
		}

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}

visualization_msgs::Marker makeSphere( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = msg.scale * 0.6;
  marker.scale.y = msg.scale * 0.6;
  marker.scale.z = msg.scale * 0.6;
  marker.color.r = 0.5;
  marker.color.g = 1.0;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::InteractiveMarkerControl& makeSphereControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeSphere(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void make6DofMarker(unsigned int interaction_mode, std::string name_str)
{

	bool show_6dof = true;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.pose.position.x = 0.0;
	int_marker.pose.position.y = 0.2;
	int_marker.pose.position.z = 0.5;
  int_marker.scale = 0.5;

  int_marker.name = name_str;
  int_marker.description = "6-DOF Control";

  // insert a sphere control
  makeSphereControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;	
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = name_str + mode_text;
  }

	visualization_msgs::InteractiveMarkerControl control;

	
  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}
//*/

int main( int argc, char** argv )
{
  ros::init(argc, argv, "MyLineMarker");
  ros::NodeHandle n;
 
	/*	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(2);

  int counter = 5;
  while (ros::ok())
  {
		make_line(marker_pub, counter);
		counter--;
		if(counter == 0) {
			counter = 5;
		}
    r.sleep();

  }
	*/

	server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls", "", false) );

  ros::Duration(0.1).sleep();

  menu_handler.insert( "First Entry", &processFeedback );
  menu_handler.insert( "Add New Interactive Marker", &processFeedback );
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

	make6DofMarker(visualization_msgs::InteractiveMarkerControl::MOVE_3D, "6DoF" + std::to_string(marker_counter));
	marker_counter++;
	server->applyChanges();
  ros::spin();
  server.reset();

}
