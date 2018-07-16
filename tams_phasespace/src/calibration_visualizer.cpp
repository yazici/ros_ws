/* calibration_visualizer.cpp
 *
 * Simple demo to visualize PhaseSpace ImpulseX2E calibration accuracy
 * in rviz. Start the PhaseSpace system and the ROS tracking node,
 * then move the calibration-wand inside the system workspace.
 * This node subscribes to the marker positions and compares them with
 * the known inter-LED distances from the wand specification, converts
 * marker distance into a color-encoded heat-map, and creates a growing
 * rviz MarkerArray with color-encoded cubes at the marker positions.
 * Optionally, the MarkerArray can be downsampled to a (x,y,z)-grid
 * of user-specified cell-size.
 * 
 * Note that we could also connect to OWL directly; however, using
 * the ROS message allows us to run from rosbags easily.
 *
 * Usage: 
 * roslaunch tams_phasespace demo1.launch
 * roslaunch tams_phasespace demo3.launch
 *
 * TODO:
 * 
 * 2017.10.18 - convert to phasespace_msgs
 * 2016.01.20 - udpates
 * 2016.01.16 - new
 *
 * (c) 2016 fnh, hendrich@informatik.uni-hamburg.de
 */

 
#include <math.h>
#include <stdio.h>
#include <pthread.h>

#include <vector>
#include <string>
#include <map>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <phasespace_msgs/MarkersStamped.h>



// #include "tams_phasespace/CalibrationVisualization.hh"
// using namespace ros;
// using namespace std;


class CalibrationVisualization {

  public:
    CalibrationVisualization();
    ~CalibrationVisualization();

    void init();
    void run();


  private:
    void createHeatmap();
    void createDistanceTable(); 
    void markersCallback( const phasespace_msgs::MarkersStamped markers );
    void createMarker( int id, geometry_msgs::Point position, std_msgs::ColorRGBA color );
    void timerCallback( const ros::TimerEvent& e );
    void publishMarkers();
    std_msgs::ColorRGBA getHeatmapColor( double dx );

    ros::NodeHandle nodeHandle;
    ros::Publisher  rvizPublisher;
    ros::Subscriber markersSubscriber;

    int             debugLevel;
    int             sequenceNumber;
    int             markerSequenceNumber;
    int             maxMarkerMessagesCount;
    std::string     parentFrame;
    double          markerRadius;
    double          cellSize;
    double          colormapScale;
    double          visualizationRate; // e.g. 1 Hz
    double          updateRate;        // typically 480 Hz

   
    std::map<int,double> distanceTable; // marker+marker -> known distance
    std::vector<phasespace_msgs::MarkersStamped> trackedMarkerMessages; // incoming tracking data
    std::vector<std_msgs::ColorRGBA> colormap; // distance -> color
    visualization_msgs::MarkerArray  rvizMarkers; // colored output markers

    pthread_mutex_t mutex;

};


CalibrationVisualization::CalibrationVisualization(): nodeHandle("~")
{

  ROS_INFO( "CalibrationVisualization <init>..." );
  init();
  // print_params();
}


CalibrationVisualization::~CalibrationVisualization()
{
  ; // empty, let the OS manage and free all resources
}


/**
 * initialize from ROS node params.
 */
void CalibrationVisualization::init()
{

  ROS_INFO( "CalibrationVisualization init()..." );
  pthread_mutex_init( &mutex, NULL ); // default attributes

  nodeHandle.param(              "debug_level",     debugLevel,   5 );
  nodeHandle.param(              "max_samples",     maxMarkerMessagesCount,  480*60*10 ); // ten minutes of data
  nodeHandle.param(              "update_rate",     updateRate, 480.0 );  // 480 Hz system default
  nodeHandle.param(         "visualization_rate",   visualizationRate,   1.0 ); // once a second
  nodeHandle.param(              "marker_radius",   markerRadius, 0.01 );
  nodeHandle.param(              "cell_size",       cellSize,     0.0  ); // 0 meaning none - NOT IMPLEMENTED YET
  nodeHandle.param(              "colormap_scale",  colormapScale, 0.01 ); // 1cm = full blue/red
  nodeHandle.param<std::string>( "parent_frame",    parentFrame,  "world" );

  createHeatmap();
  createDistanceTable();

  markersSubscriber  = nodeHandle.subscribe<phasespace_msgs::MarkersStamped>( 
                         "owl_markers", 
                         1,
                         &CalibrationVisualization::markersCallback,
                         this );
  rvizPublisher      = nodeHandle.advertise<visualization_msgs::MarkerArray>( "rviz_owl_calibration_markers", 2 );

  sequenceNumber = 0;
  markerSequenceNumber = 0;
}


/**
 * keys are (1000*marker1 + marker2), values are known
 * calibrated distance in meters.
 */
void CalibrationVisualization::createDistanceTable() {
  // this is hardcoded for the PhaseSpace calibration wand
  // with marker-IDs 4..11
  distanceTable[ 4006 ] = 0.25; // LED marker 4 -> 6
  distanceTable[ 4008 ] = 0.50;
  distanceTable[ 5007 ] = 0.25;
  distanceTable[ 6008 ] = 0.25;
  distanceTable[ 7009 ] = 0.25;
  distanceTable[ 8010 ] = 0.25;
  distanceTable[ 9011 ] = 0.25;

  // TODO: read from ros-param or command-line
}


std_msgs::ColorRGBA rgba( float r, float g, float b ) {
  std_msgs::ColorRGBA color;
  color.r = r; 
  color.g = g;
  color.b = b;
  color.a = 0.8; // 0=transparent, 1=opaque
  return color;
}


double clip( double v, double min, double max ) {
  if (v >= max) return max;
  if (v <= min) return min;
  return v;
}


void CalibrationVisualization::createHeatmap() 
{
  // Matlab 'jet'
  colormap.clear();
  colormap.push_back( rgba(          0,         0,    0.5000 )); // 0
  colormap.push_back( rgba(          0,         0,    0.5625 )); 
  colormap.push_back( rgba(          0,         0,    0.6250 ));
  colormap.push_back( rgba(          0,         0,    0.6875 ));
  colormap.push_back( rgba(          0,         0,    0.7500 ));
  colormap.push_back( rgba(          0,         0,    0.8125 ));
  colormap.push_back( rgba(          0,         0,    0.8750 ));
  colormap.push_back( rgba(          0,         0,    0.9375 ));
  colormap.push_back( rgba(          0,         0,    1.0000 ));
  colormap.push_back( rgba(          0,    0.0625,    1.0000 ));
  colormap.push_back( rgba(          0,    0.1250,    1.0000 )); // 10
  colormap.push_back( rgba(          0,    0.1875,    1.0000 )); 
  colormap.push_back( rgba(          0,    0.2500,    1.0000 ));
  colormap.push_back( rgba(          0,    0.3125,    1.0000 ));
  colormap.push_back( rgba(          0,    0.3750,    1.0000 ));
  colormap.push_back( rgba(          0,    0.4375,    1.0000 ));
  colormap.push_back( rgba(          0,    0.5000,    1.0000 ));
  colormap.push_back( rgba(          0,    0.5625,    1.0000 ));
  colormap.push_back( rgba(          0,    0.6250,    1.0000 ));
  colormap.push_back( rgba(          0,    0.6875,    1.0000 ));
  colormap.push_back( rgba(          0,    0.7500,    1.0000 )); // 20
  colormap.push_back( rgba(          0,    0.8125,    1.0000 )); 
  colormap.push_back( rgba(          0,    0.8750,    1.0000 ));
  colormap.push_back( rgba(          0,    0.9375,    1.0000 ));
  colormap.push_back( rgba(          0,    1.0000,    1.0000 ));
  colormap.push_back( rgba(     0.0625,    1.0000,    0.9375 ));
  colormap.push_back( rgba(     0.1250,    1.0000,    0.8750 ));
  colormap.push_back( rgba(     0.1875,    1.0000,    0.8125 ));
  colormap.push_back( rgba(     0.2500,    1.0000,    0.7500 ));
  colormap.push_back( rgba(     0.3125,    1.0000,    0.6875 ));
  colormap.push_back( rgba(     0.3750,    1.0000,    0.6250 )); // 30
  colormap.push_back( rgba(     0.4375,    1.0000,    0.5625 )); 
  colormap.push_back( rgba(     0.5000,    1.0000,    0.5000 )); // max green
  colormap.push_back( rgba(     0.5625,    1.0000,    0.4375 ));
  colormap.push_back( rgba(     0.6250,    1.0000,    0.3750 ));
  colormap.push_back( rgba(     0.6875,    1.0000,    0.3125 ));
  colormap.push_back( rgba(     0.7500,    1.0000,    0.2500 ));
  colormap.push_back( rgba(     0.8125,    1.0000,    0.1875 ));
  colormap.push_back( rgba(     0.8750,    1.0000,    0.1250 ));
  colormap.push_back( rgba(     0.9375,    1.0000,    0.0625 ));
  colormap.push_back( rgba(     1.0000,    1.0000,         0 ));
  colormap.push_back( rgba(     1.0000,    0.9375,         0 )); // 40
  colormap.push_back( rgba(     1.0000,    0.8750,         0 )); 
  colormap.push_back( rgba(     1.0000,    0.8125,         0 ));
  colormap.push_back( rgba(     1.0000,    0.7500,         0 ));
  colormap.push_back( rgba(     1.0000,    0.6875,         0 ));
  colormap.push_back( rgba(     1.0000,    0.6250,         0 ));
  colormap.push_back( rgba(     1.0000,    0.5625,         0 ));
  colormap.push_back( rgba(     1.0000,    0.5000,         0 ));
  colormap.push_back( rgba(     1.0000,    0.4375,         0 ));
  colormap.push_back( rgba(     1.0000,    0.3750,         0 ));
  colormap.push_back( rgba(     1.0000,    0.3125,         0 )); // 50
  colormap.push_back( rgba(     1.0000,    0.2500,         0 )); 
  colormap.push_back( rgba(     1.0000,    0.1875,         0 ));
  colormap.push_back( rgba(     1.0000,    0.1250,         0 ));
  colormap.push_back( rgba(     1.0000,    0.0625,         0 ));
  colormap.push_back( rgba(     1.0000,         0,         0 ));
  colormap.push_back( rgba(     0.9375,         0,         0 ));
  colormap.push_back( rgba(     0.8750,         0,         0 ));
  colormap.push_back( rgba(     0.8125,         0,         0 ));
  colormap.push_back( rgba(     0.7500,         0,         0 ));
  colormap.push_back( rgba(     0.6875,         0,         0 )); // 60
  colormap.push_back( rgba(     0.6250,         0,         0 )); 
  colormap.push_back( rgba(     0.5625,         0,         0 ));
  colormap.push_back( rgba(     0.5000,         0,         0 ));
}


/**
 * return the color to be used for the given distance error (in meters).
 * A zero value returns the median color from the current heat-map,
 * values larger than colormapScale return the maximum color, values
 * smaller than -colormapScale return the minium color.
 */
std_msgs::ColorRGBA CalibrationVisualization::getHeatmapColor( double dx ) 
{
  double d2 = clip( dx, -colormapScale, colormapScale );
  double NC = colormap.size();
  int    ix = (int) ((d2 / (colormapScale)) * (NC/2))  + NC/2;

  if (debugLevel > 2) {
    ROS_INFO( "-#- getHeatmapColor error=%9.4lf d2=%9.4lf scale=%9.4lf color index=%d", dx, d2, colormapScale, ix );
  }
  return colormap[ ix ];
}


void CalibrationVisualization::createMarker( int id, geometry_msgs::Point position, std_msgs::ColorRGBA color )
{
  visualization_msgs::Marker  tmp;
  {
    tmp.header.stamp = ros::Time::now(); ; // ros::Time::now(); // or ros::Time() aka t=0 for permanent visibility
    tmp.header.frame_id = parentFrame; 
    tmp.ns = "phasespace";   // no namespace for now
    // tmp.id = id; // broken: overwrites  previous marker ID
    // tmp.id = rvizMarkers.markers.size();
    tmp.id = sequenceNumber; sequenceNumber++;
    tmp.action = visualization_msgs::Marker::ADD; // aka create-or-modify
    tmp.type = visualization_msgs::Marker::CUBE;
    tmp.scale.x = markerRadius;
    tmp.scale.y = markerRadius;
    tmp.scale.z = markerRadius;
    tmp.color = color;
    tmp.pose.position = position;
    tmp.pose.orientation.x = 0;
    tmp.pose.orientation.y = 0;
    tmp.pose.orientation.z = 0;
    tmp.pose.orientation.w = 1;
    tmp.lifetime = ros::Duration( 5.0 ); // infinite, or value in seconds
  }
  rvizMarkers.markers.push_back( tmp );
}


/**
 * called when the visualizationTimer expires. This method does
 * the real work: it goes through the list of all marker messages
 * received so far, checks pairs for markers against the known-
 * distances table, and builds a heatmap color-encoded visualization
 * marker for all pairs encountered.
 * This results in huge messages, but that is what we want here.
 */
void CalibrationVisualization::timerCallback( const ros::TimerEvent& e ) 
{
  ROS_INFO( "CalViz.timerCallback triggered." );

  pthread_mutex_lock( &mutex );
  rvizMarkers.markers.clear(); // clear previous array

  if (debugLevel > 2) {
    ROS_INFO( "CalViz: iterating over %ld marker messages...", trackedMarkerMessages.size() );
  }

  for( int i=0; i< trackedMarkerMessages.size(); i++ ) {
    phasespace_msgs::MarkersStamped mm = trackedMarkerMessages[i];

    // std::vector<int> markerIDs = mm.markerIDs;
    // std::vector<geometry_msgs::Point> markerPositions = mm.markerPositions;

    if (debugLevel > 4) ROS_INFO( "-#- iteration %d", i );

    for( int j = 0; j < mm.data.size(); j++ ) {
      for( int k = j+1; k < mm.data.size(); k++ ) {

        if (debugLevel > 4) ROS_INFO( "-#- j=%d k=%d", j, k );

        int key = 1000*mm.data[j].id + mm.data[k].id;  // e.g. 4,6 -> 4006
        int found = distanceTable.count( key ); // 0 or 1

        if (debugLevel > 4) ROS_INFO( "-#- key=%d found=%d", key, found );

        if (found) {
           double dx = mm.data[j].position.x - mm.data[k].position.x;
           double dy = mm.data[j].position.y - mm.data[k].position.y;
           double dz = mm.data[j].position.z - mm.data[k].position.z;
           double measuredDistance = sqrt( dx*dx + dy*dy + dz*dz );
           double knownDistance = distanceTable[ key ];

           if (debugLevel > 4)
           ROS_INFO( "found matched pair <%d,%d> (msg index %d %d) calibrated %9.4lf measured %9.4lf",
                     mm.data[j].id, mm.data[k].id, j, k, knownDistance, measuredDistance );

           std_msgs::ColorRGBA color = getHeatmapColor( measuredDistance - knownDistance );
           createMarker( mm.data[j].id, mm.data[j].position, color );

           if (debugLevel > 4) ROS_INFO( "-#- created marker j=%d", j );
        }
      }
    }
  }
  ROS_INFO( "CalViz: created %ld visualization markers...", rvizMarkers.markers.size() );
  pthread_mutex_unlock( &mutex );

  publishMarkers();
  trackedMarkerMessages.clear(); // Eugen 
}


/**
 * called on an incoming PhaseSpaceMarkers message.
 */
void CalibrationVisualization::markersCallback( const phasespace_msgs::MarkersStamped markers )
{
  if (markerSequenceNumber < 10) {
    ROS_INFO( "CalViz.markersCallback %p", &markers );
  }
  markerSequenceNumber++;

  // hardcoded downsampling by 16
  if ((markerSequenceNumber % 16) != 0) return;

  pthread_mutex_lock( &mutex );

  if (trackedMarkerMessages.size() < maxMarkerMessagesCount) {
    trackedMarkerMessages.push_back( markers );
  }
  else { // re-alloc: throw out the oldest half of messages
    ROS_WARN( "CalViz: capacity exceeded, purging oldest marker messages..." );
    int N = maxMarkerMessagesCount/2;
    for( int i=0; i < N; i++ ) {
      trackedMarkerMessages[i] = trackedMarkerMessages[i+N];
    }
    trackedMarkerMessages.resize( N );
    trackedMarkerMessages.push_back( markers );
    ROS_WARN( "CalViz: purging and compacting finished." );
  }

  pthread_mutex_unlock( &mutex );
}


/**
 * publish the markers available in our visualizationMarkers
 * member variable.
 */
void CalibrationVisualization::publishMarkers() 
{
  ros::Time now = ros::Time::now();
  phasespace_msgs::MarkersStamped msg;
  msg.header.stamp    = now;
  msg.header.frame_id = parentFrame;
  msg.header.seq      = sequenceNumber;  sequenceNumber ++; // FIXME: should use separate counter here...

  rvizPublisher.publish( rvizMarkers );

}



/* 
 * ROS main loop
 */
void CalibrationVisualization::run()
{
  ros::Rate  loop_rate( updateRate );
  ros::Timer timer = nodeHandle.createTimer( ros::Duration( 1.0 / visualizationRate ), 
                                             &CalibrationVisualization::timerCallback,
                                             this );

  // main loop
  while( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();

  } // while
}


int main( int argc, char ** argv  ) {
  ros::init( argc, argv, "calibration_visualization", 1 ); // 1=no NoSigintHandler
  CalibrationVisualization calibrationVisualization;
  calibrationVisualization.run();
}

