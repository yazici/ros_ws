#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <cmath>
const double PI = 3.14159265358979323846;
#include <string>

#include <iostream>

class InteractiveLineMarker
{

  public:
    InteractiveLineMarker(int line_id, std::string frame_id );
    ~InteractiveLineMarker();

		void setStartPoint( geometry_msgs::Pose &pose );
		void setEndPoint( geometry_msgs::Pose &pose );

		int																line_id;
		std::string											  frame_id;
		std::string												start_point_name;
		geometry_msgs::Point      				start_point;
		std::string												end_point_name;
    geometry_msgs::Point      				end_point;

};

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
ros::Publisher marker_pub;
static int marker_counter = 1;
static int line_counter = 1;
std::map< std::string, InteractiveLineMarker* > point_line_map;

void make_line( int line_id, std::string frame_id, geometry_msgs::Point& start_point, geometry_msgs::Point& end_point);

visualization_msgs::Marker get_arrow( geometry_msgs::Pose &pose, std::string color, visualization_msgs::InteractiveMarker &msg );
visualization_msgs::InteractiveMarkerControl& makeCoodinateSystemControl( visualization_msgs::InteractiveMarker &msg );
void make6DOFMarker( std::string marker_name, std::string frame_id, geometry_msgs::Pose pose );

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

////////////////////////////////////////////////////////////////////////////////////////////////////////

InteractiveLineMarker::InteractiveLineMarker( int line_id, std::string frame_id )
{
	this->line_id = line_id;
	this->frame_id = frame_id;
}

InteractiveLineMarker::~InteractiveLineMarker()
{
}

void InteractiveLineMarker::setStartPoint( geometry_msgs::Pose &pose )
{
	start_point_name = "line_" + std::to_string(line_id) + "_start_point";
	make6DOFMarker( start_point_name, frame_id, pose );
	point_line_map.insert( std::make_pair( start_point_name, this ) );
	start_point.x = pose.position.x;
	start_point.y = pose.position.y;
	start_point.z = pose.position.z;
}

void InteractiveLineMarker::setEndPoint( geometry_msgs::Pose &pose )
{
	end_point_name = "line_" + std::to_string(line_id) + "_end_point";
	make6DOFMarker( end_point_name, frame_id, pose );
	point_line_map.insert( std::make_pair( end_point_name, this ) );
	end_point.x = pose.position.x;
	end_point.y = pose.position.y;
	end_point.z = pose.position.z;
	make_line( line_id, frame_id, start_point, end_point );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

void make_line( int line_id, std::string frame_id, geometry_msgs::Point& start_point, geometry_msgs::Point& end_point)
{
	// define the point and line_strip marker
	visualization_msgs::Marker points, line_strip;
	points.header.frame_id = line_strip.header.frame_id = frame_id;
	points.header.stamp = line_strip.header.stamp = ros::Time::now();
	points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
	points.id = 10000 + line_id;
  line_strip.id = 20000 + line_id;

	points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

	// POINTS markers use x and y scale for width/height respectively
 	points.scale.x = 0.02;
  points.scale.y = 0.02;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.03;

	// Points are red
  points.color.r = 1.0;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  points.points.push_back(start_point);
	points.points.push_back(end_point);
	line_strip.points.push_back(start_point);
  line_strip.points.push_back(end_point);

	//ros::Rate r(3);
	for ( int i = 0; i < 9; i++ )
	{
		marker_pub.publish(points);
		marker_pub.publish(line_strip);
		//r.sleep();
	}

}

////////////////////////////////////////////////////////////////////////////////////////////////////////

visualization_msgs::Marker get_arrow( geometry_msgs::Pose &pose, std::string color, visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::Marker arrow;

	arrow.type = visualization_msgs::Marker::ARROW;
	arrow.pose = pose;

 	arrow.scale.x = msg.scale * 1;
  arrow.scale.y = msg.scale * 0.04;
	arrow.scale.z = msg.scale * 0.1;

	// set the color of each arrow
  arrow.color.a = 1.0;
	if ( color == "red") {
		arrow.color.r = 1.0;
	} else if ( color == "green" ) {
		arrow.color.g = 1.0;
	} else if ( color == "blue" ) {
		arrow.color.b = 1.0;
	}

	return arrow;
}

visualization_msgs::InteractiveMarkerControl& makeCoodinateSystemControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

	geometry_msgs::Pose pose;
	tf::Quaternion q0 = tf::createQuaternionFromRPY( 0, 0, 0 );
	tf::Quaternion q = q0;
	pose.orientation.x = q0.x();
	pose.orientation.y = q0.y();
	pose.orientation.z = q0.z();
	pose.orientation.w = q0.w();
	visualization_msgs::Marker arrow_x = get_arrow(pose, "red",  msg);
	control.markers.push_back( arrow_x );

	tf::Quaternion q1 = tf::createQuaternionFromRPY( 0, 0, PI/2 );
	q0 = q*q1;
	pose.orientation.x = q0.x();
	pose.orientation.y = q0.y();
	pose.orientation.z = q0.z();
	pose.orientation.w = q0.w();
	visualization_msgs::Marker arrow_y = get_arrow(pose, "green", msg);
	control.markers.push_back( arrow_y );

	tf::Quaternion q2 = tf::createQuaternionFromRPY( 0, -PI/2, 0 );
	q0 = q*q2;
	pose.orientation.x = q0.x();
	pose.orientation.y = q0.y();
	pose.orientation.z = q0.z();
	pose.orientation.w = q0.w();
	visualization_msgs::Marker arrow_z = get_arrow(pose, "blue", msg);
	control.markers.push_back( arrow_z );

  msg.controls.push_back( control );

  return msg.controls.back();
}

void make6DOFMarker( std::string marker_name, std::string frame_id, geometry_msgs::Pose pose )
{
	visualization_msgs::InteractiveMarker int_marker;
	//default can be "world".
  int_marker.header.frame_id = frame_id;
  int_marker.pose = pose;
  int_marker.scale = 0.3;

  int_marker.name = marker_name;
  int_marker.description = marker_name;

  // insert a coodinate system control
  makeCoodinateSystemControl(int_marker);
	unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  int_marker.controls[0].interaction_mode = interaction_mode;

	visualization_msgs::InteractiveMarkerControl control;

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

	server->insert( int_marker );
	server->setCallback( int_marker.name,  &processFeedback);
	if ( interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE )
    menu_handler.apply( *server, int_marker.name );
	server->applyChanges();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

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
			if ( feedback->menu_entry_id == 1 )
			{
				server->erase(feedback->marker_name);
				std::cout << "delete button is pressed" << std::endl;
			} else if ( feedback->menu_entry_id == 2 )
			{
				// make6DofMarker( visualization_msgs::InteractiveMarkerControl::MOVE_3D, "6DoF" + std::to_string( marker_counter ) );
				// marker_counter++;
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
			if ( point_line_map.find( feedback->marker_name ) != point_line_map.end() )
			{
				std::string point_name = feedback->marker_name;
				InteractiveLineMarker* line_ptr = point_line_map[point_name];
				geometry_msgs::Pose pose = feedback->pose;
				if ( point_name == line_ptr->start_point_name )
				{
					line_ptr->start_point.x = pose.position.x;
					line_ptr->start_point.y = pose.position.y;
					line_ptr->start_point.z = pose.position.z;
					make_line( line_ptr->line_id, line_ptr->frame_id, line_ptr->start_point, line_ptr->end_point );
				} else if ( point_name == line_ptr->end_point_name )
				{
					line_ptr->end_point.x = pose.position.x;
					line_ptr->end_point.y = pose.position.y;
					line_ptr->end_point.z = pose.position.z;
					make_line( line_ptr->line_id, line_ptr->frame_id, line_ptr->start_point, line_ptr->end_point );
				}
			}
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
