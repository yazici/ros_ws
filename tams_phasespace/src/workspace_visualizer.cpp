/* workspace_visualizer.cpp
 *
 * Simple rviz visualization of the PhaseSpace ImpulseX2E workspace
 * using the current camera layout.
 *
 * This program expects that the PhaseSpace tracker has been started
 * and that the /world to /phasespace frames are configured. We then
 * listen to the (latched) FIXME owl_cameras topic to find the poses
 * (positions and orientations) of the PhaseSpace cameras.
 * It then generates regularly-spaced positions inside the given
 * workspace boudaries and counts the visibility of the given point
 * within the view-frustrum of all cameras. We approximate the view-
 * frustrum with a fixed 60-degrees angle, as PhaseSpace does not
 * provide the internal calibration data of the individual cameras.
 * Finally, we generate a Marker (CUBE_LIST) with the positions and 
 * visibility counts of the sampled points for visualization in rviz.
 *
 * Usage: 
 * roslaunch phasespace_ros  start.launch <args>
 * roslaunch tams_phasespace workspace_visualization.launch <args>
 *
 * TODO:
 * - implement "normal" cameras (Kinect, Logitech, ...)
 * - impelment different colormaps
 *
 * 2017.11.04 - play with Octomap in addition to viz markers
 * 2017.11.02 - dynamic reconfigure to enable/disable cameras
 * 2017.11.01 - first test on live system
 * 2017.10.18 - created 
 *
 * (c) 2017 fnh, hendrich@informatik.uni-hamburg.de
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
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <phasespace_msgs/RigidsStamped.h> 

#include <dynamic_reconfigure/server.h>
#include <tams_phasespace/WorkspaceParametersConfig.h>

#include <octomap/octomap.h>
#include <octomap/CountingOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapWithPose.h> // NOT SUPPORTED IN THEIR OWN RVIZ PLUGINS :-(


// #include "tams_phasespace/WorkspaceVisualization.hh"
// using namespace ros;
// using namespace std;


class WorkspaceVisualization {

  public:
    WorkspaceVisualization();
    ~WorkspaceVisualization();


    void init();
    void run();


  private:
    void createColormaps();
    void createHeatmap();
    void cameraPositionsCallback( const phasespace_msgs::RigidsStamped cameras );
    void calculateWorkspace();
    void createMarker( int id, geometry_msgs::Point position, std_msgs::ColorRGBA color );
    void publishMarkers();
    // std_msgs::ColorRGBA getHeatmapColor( double dx );
    void updateParameters( tams_phasespace::WorkspaceParametersConfig & config, uint32_t level);

    ros::NodeHandle nodeHandle;
    ros::Publisher  rvizPublisher;
    ros::Publisher  octomapPublisher;
    ros::Subscriber camerasSubscriber;
    phasespace_msgs::RigidsStamped cameras;
    std::vector<int> activeCameras; // 0=ignore 1=use

    double          workspace_xmin, workspace_xmax, workspace_dx;
    double          workspace_ymin, workspace_ymax, workspace_dy;
    double          workspace_zmin, workspace_zmax, workspace_dz;

    int             debug_level;
    int             sequenceNumber;
    int             markerSequenceNumber;
    std::string     parentFrame;
    double          markerSize;
    int             colorMapType;

    double          cellSize;
    double          colormapScale;
    double          visualizationRate; // e.g. 1 Hz
    double          updateRate;        // typically 480 Hz

    // octomap::CountingOcTree tree; COMPLETELY USELESS, CAN'T INSERT NODES 
    octomap::OcTree tree;

   
    std::vector<std_msgs::ColorRGBA> colormap; // count -> color
    std::vector<std::vector<std_msgs::ColorRGBA> > vectorOfColormaps;
    int colormapIndex;
    visualization_msgs::Marker       workspaceMarkers; // colored cubes 

    pthread_mutex_t mutex;
};


int clamp( int value, int lower, int upper ) {
  if (value <= lower) return lower;
  if (value >= upper) return upper;
  return value;
}




WorkspaceVisualization::WorkspaceVisualization(): nodeHandle("~"), tree( 0.015625 )
{
  ROS_INFO( "WorkspaceVisualization <init>..." );
  init();
  // print_params();

}


WorkspaceVisualization::~WorkspaceVisualization()
{
  ; // empty, let the OS manage and free all resources
}


/**
 * initialize from ROS node params.
 */
void WorkspaceVisualization::init()
{

  ROS_INFO( "WorkspaceVisualization init()..." );
  pthread_mutex_init( &mutex, NULL ); // default attributes

  nodeHandle.param(        "debug_level", debug_level,   5 );
  nodeHandle.param(        "update_rate", updateRate,   0.2  );  // 
  nodeHandle.param( "visualization_rate", visualizationRate,   1.0 ); // once a second
  nodeHandle.param(      "marker_size", markerSize, 0.01 );
  nodeHandle.param(          "cell_size", cellSize,     0.0  ); // 0 meaning none 
  nodeHandle.param<std::string>( "parent_frame",    parentFrame,  "world" );

  nodeHandle.param( "workspace_xmin", workspace_xmin, -1.0 );
  nodeHandle.param( "workspace_xmax", workspace_xmax,  1.0 );
  nodeHandle.param( "workspace_dx",   workspace_dx,    0.1 );
  nodeHandle.param( "workspace_ymin", workspace_ymin, -1.0 );
  nodeHandle.param( "workspace_ymax", workspace_ymax,  1.0 );
  nodeHandle.param( "workspace_dy",   workspace_dy,    0.1 );
  nodeHandle.param( "workspace_zmin", workspace_zmin, -1.0 );
  nodeHandle.param( "workspace_zmax", workspace_zmax,  1.0 );
  nodeHandle.param( "workspace_dz",   workspace_dz,    0.1 );

  // createHeatmap();
  createColormaps();  
  colormap = vectorOfColormaps[0]; // kRGBCYMw

  camerasSubscriber  = nodeHandle.subscribe<phasespace_msgs::RigidsStamped>(
                         "owl_cameras",
                         1,
                         &WorkspaceVisualization::cameraPositionsCallback, this );
  rvizPublisher      = nodeHandle.advertise<visualization_msgs::Marker>( 
                         "owl_workspace_markers", 2, true ); // latched!
  octomapPublisher   = nodeHandle.advertise<octomap_msgs::Octomap>(
                         "owl_workspace_octomap", 1, true ); // latched!

  sequenceNumber = 0;
  markerSequenceNumber = 0;
}



void WorkspaceVisualization::updateParameters( tams_phasespace::WorkspaceParametersConfig & config, uint32_t level )
{
  ROS_INFO("Reconfigure Request: x:(%7.5lf, %7.4lf..%7.4lf) y:(%7.5lf, %7.4lf..%7.4lf) z:(%7.5lf, %7.4lf..%7.4lf) '%s'",
    config.workspace_dx, config.workspace_xmin, config.workspace_xmax,
    config.workspace_dy, config.workspace_ymin, config.workspace_ymax,
    config.workspace_dz, config.workspace_zmin, config.workspace_zmax,
    config.parent_frame.c_str() );

  workspace_xmin = config.workspace_xmin;
  workspace_xmax = config.workspace_xmax;
  workspace_ymin = config.workspace_ymin;
  workspace_ymax = config.workspace_ymax;
  workspace_zmin = config.workspace_zmin;
  workspace_zmax = config.workspace_zmax;

  workspace_dx   = config.workspace_dx;
  workspace_dy   = config.workspace_dy;
  workspace_dz   = config.workspace_dz;

  debug_level = config.debug_level;
  markerSize   = config.marker_size;
  colorMapType   = config.colormap;

  // colormap
  int index = clamp( config.colormap, 0, vectorOfColormaps.size()-1 );
  colormap = vectorOfColormaps[index];
  colormapIndex = index;
  ROS_INFO( "colormap used is %d", index );

  // active cameras
  for( int i=0; i < activeCameras.size(); i++ ) {
    std::string tmp = "" + std::to_string( i );
    if (config.enabled_cameras.find( tmp ) != std::string::npos){
      activeCameras[i] = 1;
    }
    else activeCameras[i] = 0;

    ROS_INFO( "activeCameras[%d] = %d", i, activeCameras[i] );
  }

  // FIXME: need to re-calculate the Workspace now...
  calculateWorkspace();
  publishMarkers();
}




std_msgs::ColorRGBA rgba( float r, float g, float b ) {
  std_msgs::ColorRGBA color;
  color.r = r; 
  color.g = g;
  color.b = b;
  color.a = 0.8; // 0=transparent, 1=opaque
  return color;
}

std_msgs::ColorRGBA rgba( float r, float g, float b, float a ) {
  std_msgs::ColorRGBA color;
  color.r = r; 
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}




double clip( double v, double min, double max ) {
  if (v >= max) return max;
  if (v <= min) return min;
  return v;
}


/**
 * prepares and initializes our default set of colormaps.
 * Basically, map 0 (default) uses different colors for
 * the number of active cameras.
 * Colormap n=1..8 generate invisible (fully transparent dark red)
 * color for the entries i=0..7, and opaque green for entries
 * i+1..n.
 */
void WorkspaceVisualization::createColormaps() 
{
  // colormap 0: different colors k-RGB-MCY-W 
  std::vector<std_msgs::ColorRGBA> map;
    map.push_back( rgba( 1.0, 0.0, 0.0, 0.00 )); // invisible transparent red (0)
    map.push_back( rgba( 1.0, 0.0, 0.0, 0.01 )); // transparent red (1)
    map.push_back( rgba( 0.0, 0.9, 0.0, 0.50 )); // mostly transparent green (2)
    map.push_back( rgba( 0.0, 0.0, 1.0, 0.50 )); // transparent blue (3)

    map.push_back( rgba( 0.0, 1.0, 1.0, 1.00 )); // cyan
    map.push_back( rgba( 1.0, 0.0, 1.0, 1.00 )); // magenta
    map.push_back( rgba( 1.0, 1.0, 0.0, 1.00 )); // yellow
    map.push_back( rgba( 1.0, 1.0, 1.0, 1.00 )); // white
    map.push_back( rgba( 1.0, 1.0, 1.0, 1.00 )); // white
    map.push_back( rgba( 1.0, 1.0, 1.0, 1.00 )); // white
  vectorOfColormaps.push_back( map );

  // colormap 1..8: either transparent or opaque green
  //
  std_msgs::ColorRGBA inactive = rgba( 0.5, 0.0, 0.0, 0.0 );
  std_msgs::ColorRGBA active   = rgba( 0.0, 0.9, 0.0, 1.0 );
  for( int i=1; i < 8; i++ ) {
    map.clear();
    for( int j=0; j <= 9; j++ ) {
      if (j < i) map.push_back( inactive );
      else       map.push_back( active );
    }
    vectorOfColormaps.push_back( map );
  }
  
  colormapIndex = 0;
}


void WorkspaceVisualization::createHeatmap() 
{
  // Matlab 'jet'
  // modifies global variable colormap!
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
} // createHeatmap


/**
 * return the color to be used for the given distance error (in meters).
 * A zero value returns the median color from the current heat-map,
 * values larger than colormapScale return the maximum color, values
 * smaller than -colormapScale return the minium color.
std_msgs::ColorRGBA WorkspaceVisualization::getHeatmapColor( double dx ) 
{
  double d2 = clip( dx, -colormapScale, colormapScale );
  double NC = colormap.size();
  int    ix = (int) ((d2 / (colormapScale)) * (NC/2))  + NC/2;

  if (debug_level > 2) {
    ROS_INFO( "-#- getHeatmapColor error=%9.4lf d2=%9.4lf scale=%9.4lf color index=%d", dx, d2, colormapScale, ix );
  }
  return colormap[ ix ];
}
 */


void WorkspaceVisualization::createMarker( int id, geometry_msgs::Point position, std_msgs::ColorRGBA color )
{
  workspaceMarkers.points.push_back( position );
  workspaceMarkers.colors.push_back( color );
}




/**
 * called on an incoming PhaseSpace Rigids message: in our
 * case the poses of all cameras.
 */
void WorkspaceVisualization::cameraPositionsCallback( 
                           const phasespace_msgs::RigidsStamped markers )
{
  ROS_ERROR( "WorkspaceVisualization.cameraPositionsCallback %p", &markers );
  markerSequenceNumber++;

  pthread_mutex_lock( &mutex );
  cameras = markers;
  if (activeCameras.size() != markers.data.size()) {
    activeCameras.resize( markers.data.size());
    for( int i=0; i < activeCameras.size(); i++ ) {
      activeCameras[i] = 1;
    }
  }
  pthread_mutex_unlock( &mutex );

  calculateWorkspace();
  publishMarkers();

}


void WorkspaceVisualization::calculateWorkspace() {
  ROS_INFO( "WorkspaceVisualization.calculateWorkspace..." );

  pthread_mutex_lock( &mutex );
  // (re-) initialize the Marker 
  //
  workspaceMarkers.action = visualization_msgs::Marker::ADD;
  // workspaceMarkers.type = visualization_msgs::Marker::POINTS;
  workspaceMarkers.type = visualization_msgs::Marker::CUBE_LIST;
  // workspaceMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
  workspaceMarkers.id = 13; // ++sequenceNumber;
  workspaceMarkers.header.stamp = ros::Time::now();
  workspaceMarkers.header.frame_id = parentFrame;
  workspaceMarkers.header.seq = sequenceNumber;
  // workspaceMarkers.ns = "phasespace";
  workspaceMarkers.lifetime = ros::Duration( 0 ); // infinite, or value in seconds

  // workspaceMarkers.pose.position; // unused (0,0,0)
  // orientation (0,0,0,1) is default; Note that this pose is 
  // premultiplied to the POINTS/CUBE/SPHERE list members; 
  // make sure to update coordinates correspondingly if you
  // change the pose.orientation here...
  workspaceMarkers.pose.orientation.x = 0;
  workspaceMarkers.pose.orientation.y = 0;
  workspaceMarkers.pose.orientation.z = 0;
  workspaceMarkers.pose.orientation.w = 1;
  workspaceMarkers.scale.x = markerSize;
  workspaceMarkers.scale.y = markerSize;
  workspaceMarkers.scale.z = markerSize;

  // clear points and colors arrays
  //
  workspaceMarkers.points.clear();
  workspaceMarkers.colors.clear();

  // approximated PhaseSpace camera intrinsic matrix
  // tan(alpha/2) = sensor_size  / 2*focal_length
  // alpha = 60 degress = 60 * pi / 180 = 1.047
  // sensor_size = 3600 pixels (-1800 .. 1800)
  // f = sensor_size/2/tan(alpha/2) = 1039
  // 
  // (sensor_x)   (f 0 0 0)   (x)
  // (sensor_y) = (0 f 0 0) * (y)
  // (1       )   (0 0 1 0)   (z)
  //                          (1)
  //
  double a2 = 60.0 * (M_PI / 180.0);
  double f = 1800 / tan(a2/2); // 3119

  tf::Matrix3x3 intrinsic( f, 0, 0,   0, f, 0,   0, 0, 1 );
  ROS_INFO( "got Phasespace intrinsic camera matrix" );

  // get global world->phasespace transform. FIXME: use tf_listener. Hardcoded for now
  // 
  // rosrun tf tf_echo world phasespace
  // - Translation: [0.433, -0.536, 0.662]
  // - Rotation: in Quaternion [0.685, -0.148, -0.156, 0.696]
  tf::Transform world_phasespace_transform( 
             tf::Quaternion( 0.685, -0.148, -0.156, 0.696 ),
             tf::Vector3( 0.433, -0.536, 0.662 ));
  ROS_ERROR( "### world->phasespace transform (hardcoded) is (%7.4lf %7.4lf %7.4lf)  (%7.4lf %7.4lf %7.4lf %7.4lf)",
             world_phasespace_transform.getOrigin().getX(),
             world_phasespace_transform.getOrigin().getY(),
             world_phasespace_transform.getOrigin().getZ(),
             world_phasespace_transform.getRotation().getX(),
             world_phasespace_transform.getRotation().getY(),
             world_phasespace_transform.getRotation().getZ(),
             world_phasespace_transform.getRotation().getW() );

  std::vector<tf::Transform> extrinsics;
  for( int i=0; i < cameras.data.size(); i++ ) {
    phasespace_msgs::Rigid camera = cameras.data[i];

    tf::Transform pose;
    tf::poseMsgToTF( camera.pose, pose );
    extrinsics.push_back( world_phasespace_transform * pose );

    ROS_INFO( "### camera %d extrinsics (%7.4lf %7.4lf %7.4lf)  (%7.4lf %7.4lf %7.4lf %7.4lf)",
              i, 
              pose.getOrigin().getX(),
              pose.getOrigin().getY(),
              pose.getOrigin().getZ(),
              pose.getRotation().getX(),
              pose.getRotation().getY(),
              pose.getRotation().getZ(),
              pose.getRotation().getW() );
  }
  ROS_INFO( "got %lu Phasespace extrinsic camera matrices.", cameras.data.size() );

  // iterate over specified workspace boundaries and create
  // new Marker points and colors from point "visibility"
  //
  int id = 0;
  int visibility = 0;
  geometry_msgs::Point point; 
  tree.clear(); // reset octomap
  for( double x=workspace_xmin; x <= workspace_xmax; x+=workspace_dx ) {
    for( double y=workspace_ymin; y <= workspace_ymax; y+=workspace_dy ) {
      for( double z=workspace_zmin; z <= workspace_zmax; z+=workspace_dz ) {

        point.x = x;
        point.y = y;
        point.z = z;

        visibility = 0;

        for( int i=0; i < cameras.data.size(); i++ ) {
          if (activeCameras[i] == 0) continue;

        // int TTT = cameras.data.size(); if (TTT > 3) TTT = 1;
        // for( int i=0; i < TTT; i++ ) {
          // project point p into sensor frame of camera i
          // (x,y) = camera_matrix * extrinsic_matrix * world_point
          tf::Vector3 pp( x, y, z );
          tf::Vector3 p2; p2 = extrinsics[i].inverse() * pp;
          tf::Vector3 qq;
        
// FIXME: check Eugens' "sign convention"... 
          if (p2.getZ() <= 0) { // unless behind sensor or in sensor plane itself
            qq = intrinsic * p2; qq *= 1/p2.getZ();

            if (debug_level > 4) {
            ROS_INFO( "transform(%7.4lf,%7.4lf,%7.4lf) = (%7.4lf %7.4lf (%7.4lf)) pixels",
                       x, y, z, qq.getX(), qq.getY(), qq.getZ() );
            }

            // point only in camera frustrum, if in front of sensor and inside sensor size
            if ( (fabs(qq.getX()) < 1800) && (fabs(qq.getY()) < 1800)) {
              visibility++;
            }
          }
          else {
            if (debug_level > 4) ROS_INFO( "transform(%7.4lf %7.4lf %7.4lf) = invisible point", x, y, z );

            if (cameras.data.size() == 1) visibility = -1;
          }
          // FIXME;
        }

        /**
        std_msgs::ColorRGBA color;
        switch( visibility ) {
          case -1: color.r = 0.8; color.g= 0.0; color.b = 0.8; color.a = 0.20; break; // out-of-bounds

          case  0: color.r = 1.0; color.g= 0.0; color.b = 0.0; color.a = 0.05; break; // dark red

          case  1: color.r = 0.0; color.g= 0.8; color.b = 0.0; color.a = 0.40; break; // green
          case  2: color.r = 0.5; color.g= 1.0; color.b = 0.5; color.a = 0.60; break; // light green
          case  3: color.r = 1.0; color.g= 1.0; color.b = 0.0; color.a = 0.80; break; // yellow
          case  4: color.r = 0.9; color.g= 1.0; color.b = 1.0; color.a = 0.99; break; // almost white

          default: 
                  color.r = 1.0; color.g= 0.0; color.b = 1.0; color.a = 0.8; break; // error
        }
        */

        // clamp visibility
        int colorIndex = clamp( visibility, 0, colormap.size()-1 );
        createMarker( id, point, colormap[colorIndex] );

        octomap::point3d octoPoint( x, y, z );
        if ((visibility) >= colormapIndex) // hack :-)
          tree.setNodeValue( x, y, z, (float) 0.1*visibility, false );
          // tree.setNodeValue( x, y, z, (float) 1*visibility, false );
          // tree.setNodeValue( x, y, z, (float) 10*visibility, false );
/*
        octomap::CountingOcTreeNode* node = tree.updateNode( octoPoint );
        if (node != NULL) {
          node->setValue( visibility );
        }
        else ROS_ERROR( "Octomap broken: NULL key" );
*/
        
        // createMarker( id, point, getHeatmapColor( visibility ) );

        if ((debug_level > 4) || ((id % 1000) == 0)) {
          ROS_INFO( "tested point %d at (%7.4lf,%7.4lf,%7.4lf) visibility %d", 
                    id, x, y, z, visibility );
        }
        id ++;
      }
    }
  }
  pthread_mutex_unlock( &mutex );
  ROS_INFO( "WorkspaceVisualization.calculateWorkspace ok. (%d markers)", id );
}


/**
 * publish the markers available in our visualizationMarkers
 * member variable.
 */
void WorkspaceVisualization::publishMarkers() 
{
  pthread_mutex_lock( &mutex );
  rvizPublisher.publish( workspaceMarkers );

  octomap_msgs::Octomap OM;
/*
  OM.header.stamp = ros::Time::now();
  OM.header.frame_id = "world";

  OM.origin.position.x = 0;
  OM.origin.position.y = 0;
  OM.origin.position.z = 0;
  OM.origin.orientation.x = 0;
  OM.origin.orientation.y = 0;
  OM.origin.orientation.z = 0;
  OM.origin.orientation.w = 1;
*/

  bool status = octomap_msgs::fullMapToMsg( tree, OM );
  OM.header.frame_id = "world";

  octomapPublisher.publish( OM );
  pthread_mutex_unlock( &mutex );
}



/* 
 * ROS main loop
 */
void WorkspaceVisualization::run()
{
  ros::Rate  loop_rate( updateRate );
 //  ros::Timer timer = nodeHandle.createTimer( ros::Duration( 1.0 / visualizationRate ), 
  //                                            &WorkspaceVisualization::timerCallback,
   //                                           this );

  dynamic_reconfigure::Server<tams_phasespace::WorkspaceParametersConfig> server;
  dynamic_reconfigure::Server<tams_phasespace::WorkspaceParametersConfig>::CallbackType updateCallback;
  updateCallback = boost::bind(&WorkspaceVisualization::updateParameters, this, _1, _2);
  server.setCallback( updateCallback );

#ifdef DEBUG_FAKE_CAMERAS
  // self-test
  phasespace_msgs::RigidsStamped dummyCameras;
  dummyCameras.header.stamp = ros::Time::now();
  dummyCameras.header.frame_id = "world";
  phasespace_msgs::Rigid cam;
  cam.id = 17;
  cam.flags = 0x3;
  cam.pose.position.x = 1.5; 
  cam.pose.position.y = 0;
  cam.pose.position.z = -3;
  // cam.pose.orientation.x = -0.7071; // looking up
  // cam.pose.orientation.y = 0;
  // cam.pose.orientation.z = 0.7071;
  // cam.pose.orientation.w = 0;
  cam.pose.orientation.x = 0; // looking up
  cam.pose.orientation.y = 0;
  cam.pose.orientation.z = 0;
  cam.pose.orientation.w = 1;
  dummyCameras.data.push_back( cam );

  phasespace_msgs::Rigid cam2;
  cam2.id = 18;
  cam2.flags = 0x7;
  cam2.pose.position.x = 0.0; 
  cam2.pose.position.y = 1;
  cam2.pose.position.z = -3;
  cam2.pose.orientation.x = 0; // looking up
  cam2.pose.orientation.y = 0;
  cam2.pose.orientation.z = 0;
  cam2.pose.orientation.w = 1;
  dummyCameras.data.push_back( cam2 );

  phasespace_msgs::Rigid cam3;
  cam3.id = 3;
  cam3.flags = 0x7;
  cam3.pose.position.x = 0.0; 
  cam3.pose.position.y = 0;
  cam3.pose.position.z = 1;
  cam3.pose.orientation.x = -0.7071;
  cam3.pose.orientation.y = 0;
  cam3.pose.orientation.z =  0.7071;
  cam3.pose.orientation.w = 0;
  dummyCameras.data.push_back( cam3 );
#endif

  // main loop
  while( ros::ok() ) {
#ifdef DEBUG_FAKE_CAMERAS
    cameraPositionsCallback( dummyCameras );
#endif
    ros::spinOnce();
    publishMarkers();
    loop_rate.sleep();

  } // while
}


int main( int argc, char ** argv  ) {
  ros::init( argc, argv, "workspace_visualization", 1 ); // 1=no NoSigintHandler
  WorkspaceVisualization workspaceVisualization;
  workspaceVisualization.run();
}

