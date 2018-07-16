/** stylus_node.cpp - use our "Stylus" PhaseSpace motion tracking
    object for keyboard+6D interaction with ROS.

   This ROS node allows a variety of interaction tasks
   with combined keyboard input and tracking of the stylus
   object within the PhaseSpace tracker workspace.
   First, select the 'command mode' using the keyboard,
   then move/drag/rotate the stylus to perform actions:

   'ESC': (escape) cancel any ongoing command.

   'u' / 'CNTL-Z': (undo) undo the latest command (if
        possible)

   'p': (pose) move the stylus to a position, then click
        the stylus button or type 'space' to
        print/record the 6D pose of the stylus tip.

   'd': (distance) move the stylus to a first position,
        and click the button or type 'space' to mark it,
        then move the stylus to a second position,
        and click/type space to mark the end position.
        We calculate distance and rotation between
        the two poses.

   'f': (frame) click the button or type space to create a
        new tf frame corresponding to the selected pose.

   'g': (goal) click the button near a planning scene
        object to mark that object as the target for a
        pick action.

   '1'.. '5': (finger goal) click the button to mark the
        stylus tip pose as a bioik thumb/finger motion goal.
        1: FF, 2: MF, 3: RF, 4: LF, 5: thumb

   '7'.. '9': (Barrett goal) as before, but for F1, F2, F3
        or the Barrett hand (in the future).

   'r': (rotation) use to mark a 'rotation axis' affordance.
        Align the stylus tip with the top of the knob
        or rotation-lid, then rotate the stylus to
        indicate possible rotation direction and range.

   't': (trajectory) record a 6D pose trajectory.

   'o': (obstacle): move the stylus to record a 3D point
        trajectory to be used as an obstacle (collision
        object) for Moveit motion planning.
        The object is published to the planning scene
        and can be deleted again using the planning scene
        management services.

   's': (snake charmer): move the stylus and click to
        select the nearest link on the Kuka+Shadow robot.
        Then, drag the stylus and the selected link will
        follow the stylus, with collision checking against
        the current planning scene enabled.
        Can be used to teach arm, wrist, finger, and
        thumb positions without having to move the full
        weight of the robot.

   'w': (wrench): move the stylus to the target point, click,
         then move again and click to indicate a force vector,
         then move again and click to indicate a torque.


   During interaction, a corresonding interactive marker
   is shown in rviz.

   To allow recording/tracking the interaction, all
   commands are logged/published as strings to the
   /command topic.


   2017.11.04 - created

   (c) 2017, fnh, hendrich@informatik.uni-hamburg.de
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <stdio.h>

#include <string>
#include <iostream>

#include <vector>
#include <sstream>
#include <csignal>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MenuEntry.h>
#include <interactive_markers/menu_handler.h>

#include <phasespace_msgs/RigidsStamped.h>
#include <phasespace_msgs/ButtonsStamped.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "LineIteractiveMarker.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum StylusMode {
  MODE_UNKNOWN = -1,
  MODE_IDLE    = 0,

  MODE_POINTING_WAITING_FOR_POSE,
  MODE_POINTING_GOT_POSE,

  MODE_DISTANCE_WAITING_FOR_FIRST_CLICK,
  MODE_DISTANCE_WAITING_FOR_SECOND_CLICK,
  MODE_DISTANCE_GOT_SECOND_CLICK,

  MODE_CHARMER_WAITING_FOR_FIRST_CLICK,
  MODE_CHARMER_DRAGGING_TO_SECOND_CLICK,
  MODE_CHARMER_GOT_SECOND_CLICK,

  MODE_FRAME_WAITING_FOR_FIRST_CLICK,
  MODE_FRAME_GOT_FRAME_POSE,

  MODE_ROTATION_WAITING_FOR_FIRST_CLICK,
  MODE_ROTATION_WAITING_FOR_SECOND_CLICK,
  MODE_ROTATION_GOT_SECOND_CLICK,

  MODE_CANNOT_REACH
};



class StylusNode {

  public:
    StylusNode();
    ~StylusNode();

    void run();
    void shutdown();

  private:
    void rigidsCallback( const phasespace_msgs::RigidsStamped );
    void buttonCallback( const phasespace_msgs::ButtonsStamped );
    void publishStylusLog( std::string );

    void buttonPressed();
    void buttonClicked( ros::Duration timeSincePressed );
    void buttonReleased();


    void timerCallback( const ros::TimerEvent& );

    // ROS stuff
    ros::NodeHandle  nh;


    ros::Publisher            stylusCommandLogPublisher;
    ros::Publisher            stylusVizMarkerPublisher;

    ros::Subscriber           rigidsStampedSubscriber;
    ros::Subscriber           buttonsStampedSubscriber;
    ros::Timer                timer;

    std::vector<tf::Transform>  userCreatedTransforms;

    pthread_mutex_t           mutex;

    StylusMode                stylusMode;
    int                       stylusRigidsID; // phasespace rigids ID
    int                       stylusDriverID; // phasespace hardware ID of the driver
    std_msgs::Header          lastStylusHeader;
    phasespace_msgs::Rigid    lastStylusData;
    std_msgs::Header          lastButtonHeader;
    phasespace_msgs::Button   lastButtonData;
    geometry_msgs::Point      start_point;
    geometry_msgs::Point      end_point;

    double rate;
    unsigned long             seq;
    struct termios old_tio, new_tio;
};


/**
 * constructor.
 */
StylusNode::StylusNode() : seq(0) , stylusMode( MODE_UNKNOWN )
{
  pthread_mutex_init(&mutex, NULL);


  ros::NodeHandle nnh( "~" );
  // nnh.param( "debug", debug, false );
  nnh.param( "rate", rate, 120.0 );  // Hz
  nnh.param( "stylus_rigids_id", stylusRigidsID, 3 ); // stylus has id=3 in our default profile
  nnh.param( "stylus_driver_id", stylusDriverID, 2 ); // stylus microdriver has hw id=2

  ROS_INFO( "sample rate %8.4lf", rate );

  stylusCommandLogPublisher = nh.advertise<std_msgs::String>( "command_log", 1, true );
  stylusVizMarkerPublisher  = nh.advertise<visualization_msgs::MarkerArray>( "stylus_viz_markers", 1 );
  rigidsStampedSubscriber  = nh.subscribe<phasespace_msgs::RigidsStamped>( "rigids", 1,
                             &StylusNode::rigidsCallback, this );
  buttonsStampedSubscriber = nh.subscribe<phasespace_msgs::ButtonsStamped>( "buttons", 1,
                             &StylusNode::buttonCallback, this );

}


/**
 * destructor: forwards to shutdown().
 */
StylusNode::~StylusNode() {
  shutdown();
}


/**
 * resets the terminal (stdin).
 */
void StylusNode::shutdown() {
  // timer.stop();
  publishStylusLog( "StylusNode shutdown." );
  // reset termio
  tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}


/**
 * message and call generated on PhaseSpace button presses (red button)
 * on a PhaseSpace X2E microdriver. We save the received message and
 * run our trivial button state machine to generate
 * buttonPressed / buttonClicked / buttonReleased events.
 */
void StylusNode::buttonCallback( const phasespace_msgs::ButtonsStamped msg )
{
  // ROS_ERROR( "buttonCallback: IMPLEMENT ME!" );
  for( int i=0; i < msg.data.size(); i++ ) {
    if (msg.data[i].hw_id == stylusDriverID) { // that's us
      pthread_mutex_lock( &mutex );
      ros::Duration   dt = msg.header.stamp - lastButtonHeader.stamp;
      bool        b_prev = lastButtonData.pressed;
      bool        b_now  = msg.data[i].pressed;

      // save received data
      lastButtonHeader = msg.header;
      lastButtonData = msg.data[i];
      pthread_mutex_unlock( &mutex );

      // check button press / click / released
      if ((b_now != b_prev) && (b_now == true)) { buttonPressed(); }
      if ((b_now != b_prev) && (b_now == false)) { buttonReleased(); }

      // FIXME: use press-release times to generate short/long/repeated buttonClicked()
      //
      break;
    }
  }
}


void StylusNode::rigidsCallback( const phasespace_msgs::RigidsStamped msg )
{
  //ROS_INFO( "rigidsCallback...
  for( int i=0; i < msg.data.size(); i++ ) {
    if (msg.data[i].id == stylusRigidsID) { // that's us
      // ROS_INFO( "rigidsCallback: got new Stylus data..." );
      pthread_mutex_lock( &mutex );
      lastStylusHeader = msg.header;
      lastStylusData = msg.data[i];
      pthread_mutex_unlock( &mutex );
      break;
    }
  }
}


void StylusNode::publishStylusLog( std::string cmd )
{
  std_msgs::String msg;
  msg.data = cmd;
  stylusCommandLogPublisher.publish( msg );
}


/**
 * handle buttonPressed events on the Stylus.
 */
void StylusNode::buttonPressed()
{
  ROS_INFO( "buttonPressed..." );
  // ROS_ERROR( "buttonPressed: IMPLEMENT ME!!!" );

  switch( stylusMode ) {
    case MODE_IDLE:
    {
      ROS_WARN( "buttonPressed: MODE_IDLE, ignored." );
      break;
    }
    case MODE_POINTING_WAITING_FOR_POSE:
    {
       char buffer[1024];
       pthread_mutex_lock( &mutex );
       {
         geometry_msgs::Pose pose;
         pose = lastStylusData.pose;
         sprintf( buffer, "pose: xyz=(%7.4lf %7.4lf %7.4lf) xyzw=(%7.4lf %7.4lf %7.4lf %7.4lf)" ,
                          pose.position.x,
                          pose.position.y,
                          pose.position.z,
                          pose.orientation.x,
                          pose.orientation.y,
                          pose.orientation.z,
                          pose.orientation.w );
       }
       pthread_mutex_unlock( &mutex );
       publishStylusLog( buffer );
       stylusMode = MODE_POINTING_GOT_POSE;
       ROS_WARN_STREAM( "Enter Mode: MODE_POINTING_GOT_POSE" << std::endl );
       break;
    }
    case MODE_POINTING_GOT_POSE:
    {
      ROS_WARN( "buttonPressed: MODE_POINTING_GOT_POSE should not happen, switching to IDLE." );
      stylusMode = MODE_IDLE;
      break;
    }
    case MODE_DISTANCE_WAITING_FOR_FIRST_CLICK:
    {
      pthread_mutex_lock( &mutex );
      {
        std::string frame_id = lastStylusHeader.frame_id;
        geometry_msgs::Pose pose;
        pose = lastStylusData.pose;
        start_point.x = pose.position.x;
        start_point.y = pose.position.y;
        start_point.z = pose.position.z;
        InteractiveLineMarker myline( line_counter, frame_id );
        myline.setStartPoint( pose );
      }
      pthread_mutex_unlock( &mutex );
      stylusMode = MODE_DISTANCE_WAITING_FOR_SECOND_CLICK;
      ROS_WARN_STREAM( "Enter Mode: MODE_DISTANCE_WAITING_FOR_SECOND_CLICK" << std::endl );
      break;
    }
    case MODE_DISTANCE_WAITING_FOR_SECOND_CLICK:
    {
      ROS_WARN( "buttonPressed: MODE_DISTANCE_WAITING_FOR_SECOND_CLICK should not happen, switching to IDLE." );
      stylusMode = MODE_IDLE;
      break;
    }
    case MODE_DISTANCE_GOT_SECOND_CLICK:
    {
      ROS_WARN( "buttonPressed: MODE_DISTANCE_GOT_SECOND_CLICK should not happen, switching to IDLE." );
      stylusMode = MODE_IDLE;
      break;
    }
    case MODE_UNKNOWN:
    default:
         ROS_WARN( "buttonPressed: default MODE, should not happen, ignored." );
  }
}


/**
 * handle buttonReleased events on the Stylus.
 */
void StylusNode::buttonReleased()
{
  ROS_INFO( "buttonReleased..." );
  // ROS_ERROR( "buttonReleased: IMPLEMENT ME!!!" );
  switch( stylusMode ) {

    case MODE_POINTING_GOT_POSE:
    {
      pthread_mutex_lock( &mutex );
      {
        std::string frame_id = lastStylusHeader.frame_id;
        geometry_msgs::Pose pose;
        pose = lastStylusData.pose;
        make6DOFMarker( "point_" + marker_counter, frame_id, pose );
        marker_counter++;
      }
      pthread_mutex_unlock( &mutex );
      stylusMode = MODE_IDLE;
      ROS_WARN_STREAM( "Enter Mode: MODE_IDLE" << std::endl );
      break;
    }
    case MODE_DISTANCE_WAITING_FOR_SECOND_CLICK:
    {
      pthread_mutex_lock( &mutex );
      {
        std::string frame_id = lastStylusHeader.frame_id;
        geometry_msgs::Pose pose;
        pose = lastStylusData.pose;
        end_point.x = pose.position.x;
        end_point.y = pose.position.y;
        end_point.z = pose.position.z;
        std::string start_point_name = "line_" + std::to_string(line_counter) + "_start_point";
        InteractiveLineMarker* line_ptr = point_line_map[start_point_name];
        line_ptr->setEndPoint( pose );
        line_counter++;
      }
      pthread_mutex_unlock( &mutex );
      stylusMode = MODE_IDLE;
      ROS_WARN_STREAM( "Enter Mode: MODE_IDLE" << std::endl );
      break;
    }
    default:
         ROS_WARN( "StylusNode.buttonReleased: default MODE, ignored." );
  }
}


void StylusNode::buttonClicked( ros::Duration timeSincePressed )
{
  ROS_ERROR( "buttonClicked: IMPLEMENT ME!!!" );
}


/**
 * run initializes the interactive ("raw") terminal
 * session, opens the BrlAPI connection, and enters
 * the ROS main loop.
 */
void StylusNode::run() {
  ROS_INFO( "StylusNode.run() started..." );

  // connect to the default Braille device
  //

  // setup termio for raw keyboard input
  ROS_INFO( "StylusNode: initializing terminal for raw input..." );

  unsigned char c;
  tcgetattr(STDIN_FILENO, &old_tio);
  new_tio=old_tio;
  new_tio.c_lflag &=(~ICANON & ~ECHO);
  tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);
  int kfd = 0; // stdin
  int num = 0; // number of avaialable keystrokes

  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;



  ros::Rate loopRate( rate );
  bool quit = false;
  while( ros::ok() && !quit) {
    ros::spinOnce();

    // now get keyboard input, if any
    // c = getchar();
    // ROS_INFO( "calling read..." );
    if ((num = poll( &ufd, 1, 7 )) < 0) { // 7msec timeout
      ROS_INFO( "poll(): failed." );
      return;
    }
    else if (num > 0) {
      if(read(kfd, &c, 1) < 0)
      {
        ROS_ERROR( "cannot read() from stdin..." );
        return;
      }
    }
    else { // no key available
      c = 0;
    }
    // ROS_INFO( "poll/read/getchar c=%d", c );

    switch( c )
    {
      case 'q': quit = true;
                break;
      case 'p':
      {
        switch( stylusMode )
        {
          case MODE_IDLE:
          case MODE_UNKNOWN:
          {
            stylusMode = MODE_POINTING_WAITING_FOR_POSE;
            ROS_WARN_STREAM( "Enter Mode: MODE_POINTING_WAITING_FOR_POSE" << std::endl );
            break;
          }
          default:
            ROS_WARN_STREAM( "Button [p] Pressed under Wrong Mode" << stylusMode << std::endl );
        }
        break;
      }
      case 'd':
      {
        switch( stylusMode )
        {
          case MODE_IDLE:
          case MODE_UNKNOWN:
          {
            stylusMode = MODE_DISTANCE_WAITING_FOR_FIRST_CLICK;
            ROS_WARN_STREAM( "Enter Mode: MODE_DISTANCE_WAITING_FOR_FIRST_CLICK" << std::endl );
            break;
          }
          default:
            ROS_WARN_STREAM( "Button [p] Pressed under Wrong Mode" << stylusMode << std::endl );
        }
        std::cout << "we are here in d" << std::endl;
        break;
      }
      case 'f':
      {
        std::cout << "we are here in f" << std::endl;
        break;
      }
      // ......................
      default:
      {
        ; // empty
      }
    }
    loopRate.sleep();
  }
  ROS_INFO( "StylusNode: run loop finished." );

} // main



StylusNode* stylus_node_ptr;

/**
 * C-style signal handler to stop and close the BrlAPI
 * when the user hits cntl-c.
 */
void SIGINT_handler(int signal)
{
  ROS_ERROR( "StylusNode: received cntl-c (SIGINT), shutting down..." );

  if (stylus_node_ptr != NULL) {
    stylus_node_ptr->shutdown();
    usleep( 500*1000 );
  }

  ROS_ERROR( "Sigint handler completed, exiting now." );
  exit( 0 );
}

int main(int argc, char *argv[]) {
  ros::init( argc, argv, "phasespace_stylus_node", 1 ); // 1 means NoSigintHandler
  std::signal( SIGINT, SIGINT_handler );

  ros::NodeHandle n;

  server.reset( new interactive_markers::InteractiveMarkerServer("MyInteractiveMarker", "", false) );
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Duration(0.1).sleep();

  menu_handler.insert( "delete", &processFeedback );
  menu_handler.insert( "Add", &processFeedback );

  StylusNode stylus_node = StylusNode();
  stylus_node_ptr = &stylus_node;
  stylus_node.run();
  stylus_node.shutdown();
  server.reset();
}
