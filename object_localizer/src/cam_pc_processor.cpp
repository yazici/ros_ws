#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;
std::string camera_frame = "camera_color_optical_frame";

int filter_mean_k = 60;
float filter_stddev = 1.0;

// filtering an input point cloud
void filterOutliner ( PointCloudT::Ptr cloud )
{
	static pcl::StatisticalOutlierRemoval < PointT > sor;
  sor.setInputCloud ( cloud );
  sor.setMeanK ( filter_mean_k );
  sor.setStddevMulThresh ( filter_stddev );
  sor.filter ( *cloud );
}

class CamPCProcessor
{
public:

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    if ( cloud->width * cloud->height == 0 )
		{
			return;
		}

    pcl::PCLPointCloud2 pcl_pc2;
    PointCloudT::Ptr scene_cloud_ ( new PointCloudT );
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2 ( pcl_pc2, *scene_cloud_ );
		std::cout << "[" << cloud->header.stamp << "] Input point cloud has [" << scene_cloud_->width << "*" << scene_cloud_->height << " = " << scene_cloud_->width * scene_cloud_->height << "] data points" << std::endl;

		// downsampling and transforming the input point cloud
		filterOutliner ( scene_cloud_ );
		scene_cloud_->header.frame_id = camera_frame;
    // scene_cloud_->width = scene_cloud_->size();
		// scene_cloud_->height = 1;
		pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_->header.stamp );
    cloud_pub_.publish ( scene_cloud_ );
  }

  CamPCProcessor ()
  {
    std::string cloud_topic_in_ = "/camera/depth_registered/points";
    cloud_sub_ = nh_.subscribe ( cloud_topic_in_, 15, &CamPCProcessor::cloud_cb, this );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << cloud_topic_in_ );

    std::string cloud_topic_out_ = "/cam_pc_processor/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_topic_out_, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_topic_out_ );
  }

  ~CamPCProcessor () { }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
};

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "cam_pc_processor" );
	ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  CamPCProcessor cpp;
  ros::waitForShutdown ();
  return 0;
}
