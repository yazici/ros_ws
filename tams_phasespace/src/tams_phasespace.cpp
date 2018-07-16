/* tams_phasespace.cpp
 *
 * Simple ROS node to connect to the PhaseSpace ImpulseX2E motion tracking
 * server (aka OWL), and to publish all detected markers with their ID 
 * and position, as well as rviz visualization markers.
 * This node also publishes tf transforms for all detected markers.
 * 
 * Note: this code uses the OWL version 2 API, which is very different
 * from the older and more common OWL version 1.3 API.
 *
 * Usage: 
 * roslaunch tams_phasespace demo1.launch
 *
 * TODO:
 * test and improve robustness
 * implement options and dynamic reconfigure
 * implement rigid body tracking
 * try to memorize inconsistent C++ APIs :-(
 *
 * History: 
 * 2016.01.24 - access systempose as vector
 * 2016.01.22 - extract systempose from crap API
 * 2016.01.21 - lots of debugging
 * 2016.01.20 - convert back from named markers (string) to marker IDs (int)
 * 2016.01.04 - new, based on tams_hand_tracker in tams_mocap package
 *
 * (c) 2014-2016 fnh, hendrich@informatik.uni-hamburg.de
 */

 
#include <stdio.h>
#include <ros/ros.h>
#include <string>
#include <sstream>
#include "geometry_msgs/Pose.h"
#include "tams_phasespace/TamsPhaseSpaceMarkers.hh"


// using namespace ros;
// using namespace std;


TamsPhaseSpaceMarkers::TamsPhaseSpaceMarkers(): nodeHandle("~")
{

  ROS_INFO( "TamsPhaseSpaceMarkers <init>..." );
  init();
  // print_params();
}


TamsPhaseSpaceMarkers::~TamsPhaseSpaceMarkers()
{
  ; // empty, let the OS manage and free all resources
}


/**
 * fnh: in a future life I will memorize those completely stupid details 
 * about what types the dumb C++ compile can infer from source code 
 * and what not. Thanks Bjarne for the <std::string>f**king waste of time.
 */
void TamsPhaseSpaceMarkers::init()
{

  ROS_INFO( "TamsPhaseSpaceMarkers init()..." );

  nodeHandle.param<std::string>( "owl_server_name", owlServerName, "localhost" );
  nodeHandle.param<std::string>( "owl_options",     owlOptions, "slave=1" );
  nodeHandle.param( "owl_frequency",   owlFrequency, 480.0 );

  nodeHandle.param( "marker_radius",   markerRadius, 0.01 );
  nodeHandle.param<std::string>( "parent_frame",    parentFrame, "world" );
  nodeHandle.param( "debug_level",     debugLevel, 5 );

  sequenceNumber     = 0;

  markersPublisher   = nodeHandle.advertise<tams_phasespace::PhaseSpaceMarkers>( "owl_markers", 2 );
  rvizPublisher      = nodeHandle.advertise<visualization_msgs::MarkerArray>( "rviz_led_markers", 2 );
  cameraPublisher    = nodeHandle.advertise<visualization_msgs::MarkerArray>( "rviz_camera_markers", 2 );

}



/**
 * establish connection to the OWL server and set the relevant
 * options. Returns 0 on success, negative values on failure.
 */
int TamsPhaseSpaceMarkers::owlConnect() 
{
  if (owl.open( owlServerName ) <= 0) {
    ROS_ERROR( "-E- TamsPhaseSpaceMarkers: owlConnect failed, could not connect to server" );
    return OWL_OPEN_FAILURE;
  }

  if (owl.initialize( owlOptions ) <= 0) 
    return OWL_INITIALIZE_FAILURE;

  owl.frequency( owlFrequency );
  owl.streaming( true );

  // FNH, 2016.01.23: try accessing as std::vector as suggested by Ken Anant:
  std::vector<float> vsp = owl.property( "systempose" );
  ROS_WARN( "recieved systempose as a vector: address %p length %lu", &vsp, vsp.size() );
  if (vsp.size() == 7) {
    ROS_WARN( "systempose vector is (%7.4f %7.4f %7.4f)  (%7.4f %7.4f %7.4f %7.4f)",
              vsp[0], vsp[1], vsp[2], vsp[3], vsp[4], vsp[5], vsp[6] );
  }

  const float* sp = owl.property<const float*>( "systempose" );
  // broken std::vector<float> sp = owl.property<float>( "systempose" );
  if (sp == NULL) {
    ROS_WARN( "OWL systempose returned NULL, using (0 0 0) (0 0 0 1) instead!" );
    systemPose.position.x = 0;
    systemPose.position.y = 0;
    systemPose.position.z = 0;
    systemPose.orientation.x = 0;
    systemPose.orientation.y = 0;
    systemPose.orientation.z = 0;
    systemPose.orientation.w = 1;
  }
  else {
    // note that PhaseSpace uses quat=wxyz while ROS wants xyzw
    // ROS_WARN( "OWL systempose is xyz=(%7.4f,%7.4f,%7.4f) quat=(%7.4f,%7.4f,%7.4f,%7.4f)",
    //            sp[0], sp[1], sp[2], sp[4], sp[5], sp[6], sp[3] );
    // the f**king values disappear when accessing them again; complete crap!!!
    // [ INFO] [1453477568.628106426]: TamsPhaseSpaceMarkers init()...
// [ WARN] [1453477568.644503294]: OWL systempose is xyz=( 0.0000, 0.0000,250.8268) quat=(-0.0231, 0.0597,-0.9405,-0.3338)
// [ WARN] [1453477568.644601981]: OWL systempose is xyz=( 0.0000, 0.0000, 0.0000) quat=(   -nan, 0.0597,216602533604626010210304.0000, 0.0000)

    float x,y,z,qx,qy,qz,qw;
    x  = sp[0] / 1000;
    y  = sp[1] / 1000;
    z  = sp[2] / 1000; 
    qx = sp[4];
    qy = sp[5];
    qz = sp[6];
    qw = sp[3];

    ROS_WARN( "OWL systempose is xyz=(%7.4f,%7.4f,%7.4f) quat=(%7.4f,%7.4f,%7.4f,%7.4f)",
               x, y, z, qx, qy, qz, qw );
    ROS_WARN( "OWL systempose is xyz=(%7.4f,%7.4f,%7.4f) quat=(%7.4f,%7.4f,%7.4f,%7.4f)",
               x, y, z, qx, qy, qz, qw );


    systemPose.position.x    = x;
    systemPose.position.y    = y;
    systemPose.position.z    = z;
    systemPose.orientation.x = qx;
    systemPose.orientation.y = qy;
    systemPose.orientation.z = qz;
    systemPose.orientation.w = qw;

    ROS_ERROR( "ROS systempose is xyz=(%lf %lf %lf) quat=(%lf %lf %lf %lf)",
           systemPose.position.x,
           systemPose.position.y,
           systemPose.position.z,
           systemPose.orientation.x,
           systemPose.orientation.y,
           systemPose.orientation.w,
           systemPose.orientation.w );

    ROS_WARN( "OWL systempose is xyz=(%7.4f,%7.4f,%7.4f) quat=(%7.4f,%7.4f,%7.4f,%7.4f)",
               x, y, z, qx, qy, qz, qw );

    // sleep( 10 );
  }

  return 0; 
}



int TamsPhaseSpaceMarkers::owlReset() 
{

  ROS_WARN( "owlReset ignored, NOT YET IMPLEMENTED!" );
  return OWL_IMPLEMENT_ME_FAILURE;
  // return 0; // ok
}



void TamsPhaseSpaceMarkers::publish_markers() 
{
  // the actual tracked markers
  ros::Time now = ros::Time::now();
  tams_phasespace::PhaseSpaceMarkers msg;
  msg.header.stamp    = now;
  msg.header.frame_id = parentFrame;
  msg.header.seq      = sequenceNumber;  sequenceNumber ++;

  msg.markerIDs = markerIDs;
  msg.markerPositions = markerPositions;
  msg.markerConditions = markerConditions;
  markersPublisher.publish( msg );

  // one rviz visualiation-marker sphere per marker.
  visualization_msgs::MarkerArray  rvizMarkers;
  for( unsigned int i=0; i < markerIDs.size(); i++ ) { // a set of SPHERE markers
    visualization_msgs::Marker  tmp;
    tmp.header.stamp = now; // or ros::Time() aka t=0 for permanent visibility
    tmp.header.frame_id = parentFrame; 
    tmp.ns = "phasespace";   // no namespace for now
    tmp.id = i;
    tmp.action = visualization_msgs::Marker::ADD; // aka create-or-modify
    tmp.type = visualization_msgs::Marker::SPHERE;
    tmp.scale.x = markerRadius;
    tmp.scale.y = markerRadius;
    tmp.scale.z = markerRadius;
    tmp.color.a = 1; // opaque
    tmp.color.r = 0.9;
    tmp.color.g = 0.0;
    tmp.color.b = 0.0;
    tmp.pose.position.x = markerPositions[i].x;
    tmp.pose.position.y = markerPositions[i].y;
    tmp.pose.position.z = markerPositions[i].z;
    tmp.pose.orientation.x = 0;
    tmp.pose.orientation.y = 0;
    tmp.pose.orientation.z = 0;
    tmp.pose.orientation.w = 1;
    tmp.lifetime = ros::Duration( 5.0 ); // infinite, or value in seconds

    rvizMarkers.markers.push_back( tmp );
  }
  rvizPublisher.publish( rvizMarkers );


  // one rviz Cube visualiation-marker sphere per camera
  visualization_msgs::MarkerArray  rvizCameraMarkers;
  for (std::map<int,geometry_msgs::Pose>::iterator it=cameraPoses.begin(); it!=cameraPoses.end(); ++it) 
  {
    int index = it->first;

    visualization_msgs::Marker  tmp;
    tmp.header.stamp = now; // or ros::Time() aka t=0 for permanent visibility
    tmp.header.frame_id = parentFrame; 
    tmp.ns = "phasespace_cameras";   // no namespace for now
    tmp.id = (index+1);
    tmp.action = visualization_msgs::Marker::ADD; // aka create-or-modify
    tmp.type = visualization_msgs::Marker::CUBE;
    tmp.scale.x = 0.106; // actual camera size
    tmp.scale.y = 0.057;
    tmp.scale.z = 0.092; // note +z is rear of camera (behind camera)
    tmp.color.a = 1; // opaque
    tmp.color.r = 0.0; // black
    tmp.color.g = 0.0;
    tmp.color.b = 0.0;
    tmp.pose = it->second; 
    tmp.lifetime = ros::Duration( 5.0 ); // infinite, or value in seconds

    rvizCameraMarkers.markers.push_back( tmp );
  }
  cameraPublisher.publish( rvizCameraMarkers );

  // also, publish TF transforms for all detected markers
  //
  tf::Transform transform;
  tf::Quaternion quaternion( 0, 0, 0, 1 );
  for( unsigned int i=0; i < markerIDs.size(); i++ ) { // one TF frame per marker
    char buffer[256]; sprintf( buffer, "led_%d", markerIDs[i] );
    std::string markerFrame = buffer;

    transform.setOrigin( tf::Vector3( markerPositions[i].x, markerPositions[i].y, markerPositions[i].z ));
    transform.setRotation( quaternion );
    transformBroadcaster.sendTransform(
       tf::StampedTransform( transform, now, parentFrame, markerFrame ));
  }

  // also, (re-) publish the camera TFs
  //
  for (std::map<int,geometry_msgs::Pose>::iterator it=cameraPoses.begin(); it!=cameraPoses.end(); ++it) 
  {
    // std::cout << it->first << " => " << it->second << '\n';
    int index = it->first;
    geometry_msgs::Pose pose = it->second;

    // Note that PhaseSpace camera->id starts at zero, but PhaseSpace web GUI
    // counts from 1...
    char buffer[256]; sprintf( buffer, "camera_%d", (index+1) ); 
    std::string markerFrame = buffer;

    tf::Quaternion quaternion( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w );
    transform.setOrigin( tf::Vector3( pose.position.x, pose.position.y, pose.position.z ));
    transform.setRotation( quaternion );
    transformBroadcaster.sendTransform(
       tf::StampedTransform( transform, now, parentFrame, markerFrame ));
  }
  
  // finally, (re-) publish the global frame, aka "systempose"
  // 
  {
    //ROS_WARN( "HUGO systemPose (%f %f %f) (%f %f %f %f)",
    //          systemPose.position.x, systemPose.position.y, systemPose.position.z,
    //          systemPose.orientation.x, systemPose.orientation.y, systemPose.orientation.z, systemPose.orientation.w );

    tf::Quaternion quaternion( 
        systemPose.orientation.x, 
        systemPose.orientation.y, 
        systemPose.orientation.z, 
        systemPose.orientation.w );
    transform.setRotation( quaternion );
    transform.setOrigin( tf::Vector3( systemPose.position.x, systemPose.position.y, systemPose.position.z ));
    transformBroadcaster.sendTransform(
       tf::StampedTransform( transform, now, parentFrame, "phasespace_systempose" ));
  }

}


/*
visualization_msgs::Marker TamsPhaseSpaceMarkers::makeArrow( string from, string to, float r, float g, float b ) {
  int i = 0; // marker_map[from];
  int j = 42; // marker_map[to];

  visualization_msgs::Marker  tmp;
  tmp.header.stamp = Time::now(); // or ros::Time() aka t=0 for permanent visibility
  tmp.header.frame_id = parentFrame;
  tmp.ns = "";   // no namespace
  tmp.action = visualization_msgs::Marker::ADD; // aka create-or-modify
  tmp.type = visualization_msgs::Marker::ARROW;
  tmp.id = 0xdeadbeef + 100*i + 1*j;

  // tmp.pose.position.x = marker_poses[i].x();
  // tmp.pose.position.y = marker_poses[i].y();
  // tmp.pose.position.z = marker_poses[i].z();
  tmp.pose.position.x = 0;
  tmp.pose.position.y = 0;
  tmp.pose.position.z = 0;
  // tmp.pose.orientation.x = marker_orientations[i].x();
  // tmp.pose.orientation.y = marker_orientations[i].y();
  // tmp.pose.orientation.z = marker_orientations[i].z();

  // You can also specify a start/end point for the arrow, using the points member. 
  // If you put points into the points member, it will assume you want to do things this way.
  // The point at index 0 is assumed to be the start point, and the point at index 1 
  // is assumed to be the end.
  // scale.x is the shaft diameter, and scale.y is the head diameter. 
  // If scale.z is not zero, it specifies the head length. 
  tmp.scale.x = 0.030;
  tmp.scale.y = 0.030;
  tmp.scale.z = 0.005;

  tmp.color.a = 1; // opaque
  tmp.color.r = r;
  tmp.color.g = g;
  tmp.color.b = b;

  tmp.points.resize(2);
  tmp.points[0].x = marker_poses[i].x(); // from
  tmp.points[0].y = marker_poses[i].y();
  tmp.points[0].z = marker_poses[i].z();
  tmp.points[1].x = marker_poses[j].x(); // to
  tmp.points[1].y = marker_poses[j].y(); // to
  tmp.points[1].z = marker_poses[j].z(); // to

  tmp.lifetime = ros::Duration( 5.0 ); // infinite, or value in seconds

  return tmp;
}
*/


/* 
 * ROS main loop
 */
void TamsPhaseSpaceMarkers::run()
{
  ros::Time time;
  bool haveNewMarkers = false;

  // std::map<int,tf::Quaternion> cameraPoses;
  // std::map<int,geometry_msgs::Pose> cameraPoses;

  ros::Rate loop_rate( owlFrequency );

  // main loop
  while(ros::ok() && owl.isOpen() && owl.property<int>( "initialized" )) {
    const OWL::Event *event = owl.nextEvent(1000);
    if(!event) continue;

    switch(event->type_id()) 
    {
      case OWL::Type::ERROR: 
           {
             std::string s;
             if (event->get(s)) std::cerr << event->name() << ": " << s << std::endl;
           }
           break;

      case OWL::Type::FLOAT:
           {
             if (strcmp( event->type_name(), "float") == 0) {
               for (const float *data = event->begin(); data != event->end(); data++ ) 
               {
                  ROS_WARN( "received float event (systempose): %9.4e",  *data ); 
               }
             }
           }
           break;

      case OWL::Type::CAMERA:
           {
             for (const OWL::Camera* camera = event->begin(); camera != event->end(); camera++ ) 
               {
                  if (debugLevel >= 1) ROS_WARN( "received camera event..." );
                  geometry_msgs::Pose pose;
                  pose.position.x = camera -> pose[0] / 1000;
                  pose.position.y = camera -> pose[1] / 1000;
                  pose.position.z = camera -> pose[2] / 1000;

                  pose.orientation.x = camera -> pose[4];
                  pose.orientation.y = camera -> pose[5];
                  pose.orientation.z = camera -> pose[6];
                  pose.orientation.w = camera -> pose[3];
                  
                  cameraPoses[ camera->id ] = pose;
               }
           }
           break;

      case OWL::Type::FRAME:
           {
             if (debugLevel >= 3) {
               std::cout << "time=" << event->time() << " " << event->type_name() 
                    << " " << event->name() << "=" << event->size<OWL::Event>() 
                    << ":" << std::endl;
             }

             for (const OWL::Event *e = event->begin(); e != event->end(); e++) 
             {
                switch( e->type_id()) 
                {
                  case OWL::Type::MARKER:
                       {
                         if(e->get(markers) > 0)
                         {
                           if (debugLevel >= 3 ) {
                             std::cout << " " << e->type_name() << " " << e->name() << "=" << markers.size() << ":" << std::endl;
                             for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++) {
                               if (m->cond > 0)
                                 std::cout << "  " << m->id << ") " << m->x << " " << m->y << " " << m->z << std::endl;
                             }
                           }

                           time = ros::Time::now();
                           // markerIDs.clear();
                           markerIDs.clear();
                           markerPositions.clear();
                           markerConditions.clear();

                           for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++) {
                             if (m->cond > 0)
                             {
                               markerIDs.push_back( m->id );
                               // markerNames.push_back( std::to_string( m->id ));
                               geometry_msgs::Point p;
                               p.x = m->x/1000.0;
                               p.y = m->y/1000.0;
                               p.z = m->z/1000.0;
                               markerPositions.push_back( p );
                               markerConditions.push_back( m->cond );
                             }
                           }
                           haveNewMarkers = true;
                         }
                       }
                   break;
             } // switch type_id
           }
      }
      break;
    } // switch type_id

    if (haveNewMarkers) publish_markers();
    ros::spinOnce();
    loop_rate.sleep();

  } // while

  owl.done();
}


int main( int argc, char ** argv  ) {
  ros::init( argc, argv, "hand_markers", 1 ); // 1=no NoSigintHandler
  TamsPhaseSpaceMarkers hand_markers;

  // we use ROS params now, not command-line arguments 
  // if (argc > 1) { 
  //  hand_markers.parse_args( argc, argv );
  //}

  hand_markers.owlConnect();
  hand_markers.run();
  // hand_markers.play_xml_file();
}

