#include <stdio.h>
#include <iostream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// converse header file between ros points2 and pcl point cloud
#include <pcl_conversions/pcl_conversions.h>
#include "object_localizer_msg/BBox_int.h"
#include "object_localizer_msg/BBox_float.h"
#include "object_localizer_msg/BBox_list.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

// PCL head files
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

// boost geometry head files
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <vector>
#include <deque>

namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> point_t_b;
typedef bg::model::box<point_t_b> box_t_b;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

std::string camera_frame = "camera_depth_optical_frame";

// function used to show the merged point cloud and calculated surface normal
void Visualize ( PointCloudT::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals )
{
	std::printf ( "Visualizing point clouds...\n" );
	pcl::visualization::PCLVisualizer viewer ( "Merged Point Clouds" );
	int v1(0);
	viewer.createViewPort ( 0.0, 0.0, 1.0, 1.0, v1 );

	// The color we will be using
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// ICP aligned point cloud
	pcl::visualization::PointCloudColorHandlerRGBField< PointT > cloud_in_color_i ( cloud );
	viewer.addPointCloud ( cloud, cloud_in_color_i, "cloud_icp_v1", v1 );
	viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp_v1" );
	//Add normals
	viewer.addPointCloudNormals < pcl::PointXYZRGB, pcl::Normal > ( cloud, normals, 10, 0.01, "normals" );
	// Set background color
	viewer.setBackgroundColor ( bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1 );
	// Set camera position and orientation
	viewer.setCameraPosition ( -0.0611749, -0.040113, 0.00667606, -0.105521, 0.0891437, 0.990413 );
	viewer.setSize ( 1280, 1024 ); // Visualiser window size

	// Display the visualiser
	while ( !viewer.wasStopped () )
  {
    viewer.spinOnce ();
	}
}

class BoxSegmenter
{
public:

  void get_surface_normal( PointCloudT::Ptr cloud )
  {
    // compute surface normals
  	std::printf( "Computing surface normals...\n" );
  	// Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation< PointT, pcl::Normal > ne;
    ne.setInputCloud ( cloud );

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree< PointT >::Ptr tree ( new pcl::search::KdTree< PointT > () );
    ne.setSearchMethod ( tree );
    // Output datasets
    pcl::PointCloud< pcl::Normal >::Ptr cloud_normals ( new pcl::PointCloud< pcl::Normal > );
    // Use all neighbors in a sphere of radius 2cm
    ne.setRadiusSearch ( 0.02 );
    // Compute the features
    ne.compute ( *cloud_normals );

  	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
  	std::printf( "cloud size: %lu; normals size: %lu\n", cloud->points.size(), cloud_normals->points.size() );
  	//Visualize( cloud, cloud_normals );
  }

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    // if input cloud is empty, return
    if ( ( cloud->width * cloud->height ) == 0 )
      return;

    // convert input ros cloud to pcl cloud and save it in local variable saved_cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2( pcl_pc2, *saved_cloud );
    // save the time for saved point cloud
    sample_time = cloud->header.stamp;
  }

  void bbox_cb ( const object_localizer_msg::BBox_list::ConstPtr& bbox_list )
  {
    if ( bbox_list->BBox_list_float.size() > 0 )
    {
      // step 1, create the bounding box list
      std::vector<box_t_b> box_t_b_list;
      for ( object_localizer_msg::BBox_float bbox : bbox_list->BBox_list_float )
      {
        std::cout << "Bounding box has [x1, x2, y1, y2]: [" << bbox.x1 << "," << bbox.x2 << "," << bbox.y1 << "," << bbox.y2  << "]" << std::endl;
        box_t_b box_n { {bbox.x1, bbox.y1}, {bbox.x2, bbox.y2} } ;
        box_t_b_list.push_back( box_n );
      }

      // step 2, for each point in saved cloud, check whether they are in the bounding boxes.
      PointCloudT::Ptr cropped_cloud ( new PointCloudT );
      int point_counter = 0;
      for ( PointT temp_point: saved_cloud->points )
      {
        float x = temp_point.x;
        float y = temp_point.y;
        float z = temp_point.z;
        point_t_b temp_point_2D( x, y );
				int box_counter = 0;
        for ( box_t_b box_tmp : box_t_b_list )
        {
          if ( bg::within( temp_point_2D, box_tmp ) )
          {
						PointT new_point;
						new_point.x = x;
      			new_point.y = y;
			      new_point.z = z;
						uint8_t r = 0, g = 0, b = 0;
						r = box_counter % 5 * 50;
						g = box_counter % 3 * 80;
						b = box_counter % 7 * 30;
						uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
						new_point.rgb = *reinterpret_cast<float*>( &rgb );
            cropped_cloud->points.push_back( new_point );
            point_counter++;
          }
					box_counter++;
        }
      }
      cropped_cloud->width = point_counter;
      cropped_cloud->height = 1;
      std::cout << "cropped_cloud has " << cropped_cloud->size()  << std::endl;
      cropped_cloud->header.frame_id = "world";
      pcl_conversions::toPCL ( ros::Time::now(), cropped_cloud->header.stamp );
      cloud_pub_.publish ( cropped_cloud );
      get_surface_normal( cropped_cloud );
    }
  }

  BoxSegmenter () : saved_cloud ( new PointCloudT ), cloud_topic_ ( "/point_cloud_merger/points" ), bbox_topic_ ( "/rough_localizer/bbox_list" )
  {
    cloud_sub_ = nh_.subscribe ( cloud_topic_, 30, &BoxSegmenter::cloud_cb, this );
    std::string cloud_in_name = nh_.resolveName ( cloud_topic_ );
    ROS_INFO_STREAM ( "Listening for point cloud on topic: " << cloud_in_name );

    std::string cloud_out_name = "/box_segmenter/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_out_name, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud on topic: " << cloud_out_name );

    bbox_sub_ = nh_.subscribe ( bbox_topic_, 30, &BoxSegmenter::bbox_cb, this );
    std::string bbox_in_name = nh_.resolveName ( bbox_topic_ );
    ROS_INFO_STREAM ( "Listening for bounding box list on topic: " << bbox_in_name );
  }

  ~BoxSegmenter () { }

private:

  ros::NodeHandle nh_;
  PointCloudT::Ptr saved_cloud;
  std::string cloud_topic_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
  std::string bbox_topic_;
  ros::Subscriber bbox_sub_;
  ros::Time sample_time;
};

int main( int argc, char** argv )
{
  ros::init ( argc, argv, "box_segmenter" );
  // ros::NodeHandle n;
  BoxSegmenter BS;
  ros::spin ();
  return 0;
}
