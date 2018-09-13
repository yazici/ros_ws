#include "LineIteractiveMarker.h"

/*
** Test the function for drawing lines and IteractiiveMarker
*/
int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "stylus_node_marker_test" );
  ros::NodeHandle nh_;

	server.reset ( new interactive_markers::InteractiveMarkerServer ( "stylus_node_marker_server", "", false ) );
	marker_pub = nh_.advertise< visualization_msgs::Marker > ( "stylus_node_marker", 10 );
  menu_handler.insert( "delete", &processFeedback );
  menu_handler.insert( "Add", &processFeedback );

  geometry_msgs::Pose pose;
  std::cout << "Add new 6DOF point, enter: 0\n"
            << "Add new line, enter: 1\n"
            << "Otherwise, enter: -1\n";
  int input_num;
  std::cin >> input_num;

  if ( input_num == 0 )
  {
    pose.position.x = 1.0;
  	pose.position.y = 1.0;
  	pose.position.z = 1.0;
  	pose.orientation.w = 1.0;
  	make6DOFMarker ( "point" + std::to_string(marker_counter), "world", pose );
  	marker_counter ++;
  } else if ( input_num == 1 )
  {
    {
      lineMarkerPtr line_ptr ( new InteractiveLineMarker ( line_counter, "world" ) );
      pose.position.x = 0.5;
    	pose.position.y = 0.3;
    	pose.position.z = 1.5;
      line_ptr->setStartPoint ( pose );
      point_line_map.insert( std::make_pair ( InteractiveLineMarker::getStartPointName ( line_counter ), line_ptr ) );
    }
    {
      lineMarkerPtr line_ptr = point_line_map [ InteractiveLineMarker::getStartPointName ( line_counter ) ];
      pose.position.x = 1.5;
    	pose.position.y = 0.8;
    	pose.position.z = 1.5;
      line_ptr->setEndPoint( pose );
      point_line_map.insert( std::make_pair ( InteractiveLineMarker::getEndPointName ( line_counter ), line_ptr ) );
      line_counter++;
    }
  }

  ros::spin();
  server.reset();
}
