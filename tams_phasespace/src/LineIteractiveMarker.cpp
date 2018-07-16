#include "LineIteractiveMarker.h"

/*
** Test the function for drawing lines and IteractiiveMarker
*/
int main( int argc, char** argv )
{
  ros::init( argc, argv, "MyLineMarker" );
  ros::NodeHandle n;

	server.reset( new interactive_markers::InteractiveMarkerServer( "MyInteractiveMarker", "", false ) );
	marker_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
	ros::Duration(1).sleep();

  menu_handler.insert( "delete", &processFeedback );
  menu_handler.insert( "Add", &processFeedback );

	geometry_msgs::Pose pose;
	pose.position.x = 1.0;
	pose.position.y = 1.0;
	pose.position.z = 1.0;
	pose.orientation.w = 1.0;
	make6DOFMarker( "point" + std::to_string(marker_counter), "world", pose );
	marker_counter++;

	InteractiveLineMarker myline( line_counter, "world" );
	pose.position.x = 0.5;
	pose.position.y = 0.3;
	pose.position.z = 1.5;
	myline.setStartPoint( pose );
	pose.position.x = 1.5;
	pose.position.y = 0.8;
	pose.position.z = 1.5;
	myline.setEndPoint( pose );
	make_line( myline.line_id, myline.frame_id, myline.start_point, myline.end_point );
	line_counter++;

  ros::spin();
  server.reset();

}
