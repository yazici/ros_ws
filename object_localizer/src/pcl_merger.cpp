#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
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
std::string camera_frame = "camera_depth_optical_frame";
ros::Time sample_time;

void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
	// std::printf( "Downsampling point clouds...\n" );
  static pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.005f, 0.005f, 0.005f );
  grid.filter ( *cloud_sampled );
	std::printf( "Downsampled cloud size is %d, %d\n", cloud_sampled->width, cloud_sampled->height );
}

class PointCloudMerger
{
public:

	void transform_point_cloud ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_out )
	{
	  tf::StampedTransform transform;
		bool is_lookuped = false;
		while ( is_lookuped == false )
		{
			try
			{
				// ros::Time(0) or sample_time
		    listener.lookupTransform( "world", camera_frame, sample_time, transform );
				is_lookuped = true;
		  }
		  catch ( tf::TransformException ex )
		  {
				if ( boost::starts_with ( ex.what(), "Lookup would require extrapolation into the future." ) )
				{
					// wait for 0.05 second every time
					ros::Duration ( 0.05 ) .sleep ();
				} else
				{
					ROS_ERROR ( "%s", ex.what() );
					return;
				}
		  }
		}

		// std::cout << "tf time difference is " << ros::Time::now() - sample_time << std::endl;
		tf::Vector3 point(0, 0, 0);
		tf::Vector3 point_n(0, 0, 0);
		// std::cout << "input point cloud has " << cloud->size() << " points" << std::endl;
		for ( PointT temp_point : cloud->points )
		{
			point.setX( temp_point.x );
			point.setY( temp_point.y );
			point.setZ( temp_point.z );
			tf::Vector3 point_n = transform * point;
			temp_point.x = point_n.getX();
			temp_point.y = point_n.getY();
			temp_point.z = point_n.getZ();
			cloud_out->points.push_back (temp_point);
		}
	}

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    // if input cloud is empty or is not dense, publish the old point cloud and return
    if ( ( cloud->width * cloud->height ) == 0 && ! cloud->is_dense )
		{
			pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_total->header.stamp );
	    cloud_pub_.publish ( scene_cloud_total );
			return;
		}

    // convert input ros cloud to pcl cloud
    // and save it in local variable scene_cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2 ( pcl_pc2, *scene_cloud_ );
		sample_time = cloud->header.stamp;
		std::cout << "[" << sample_time << "] Input point cloud has [" << scene_cloud_->width << "*" << scene_cloud_->height << " = " << scene_cloud_->width * scene_cloud_->height << "] data points" << std::endl;

		// downsampling and transforming the input point cloud
		// filterOutliner ( scene_cloud_ );
		PointCloudT::Ptr scene_cloud_sampled	(new PointCloudT);
		downSampling ( scene_cloud_, scene_cloud_sampled );
		PointCloudT::Ptr scene_cloud_world	(new PointCloudT);
		transform_point_cloud ( scene_cloud_sampled, scene_cloud_world );
		std::cout << "Input point cloud after downSampling has [" << scene_cloud_world->size() << "] data points" << std::endl;

		// merging the old point cloud with the input one
		*scene_cloud_total += *scene_cloud_world;
		scene_cloud_total->header.frame_id = "world";
    scene_cloud_total->width = scene_cloud_total->size() + scene_cloud_world->size();
		scene_cloud_total->height = 1;
		std::cout << "Merged point cloud has [" << scene_cloud_total->size() << "] data points" << std::endl;
		pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_total->header.stamp );
		PointCloudT::Ptr scene_cloud_total_temp	( new PointCloudT );
		downSampling ( scene_cloud_total, scene_cloud_total_temp );
		std::cout << "Merged point cloud after downsampling has [" << scene_cloud_total_temp->size() << "] data points" << std::endl;
		scene_cloud_total = scene_cloud_total_temp;
    cloud_pub_.publish ( scene_cloud_total );
  }

  PointCloudMerger () : scene_cloud_ ( new pcl::PointCloud< PointT > ), scene_cloud_total ( new pcl::PointCloud< PointT > ) , cloud_topic_ ( "/camera/depth_registered/points" )
  {
    cloud_sub_ = nh_.subscribe ( cloud_topic_, 200, &PointCloudMerger::cloud_cb, this );
    std::string r_ct = nh_.resolveName ( cloud_topic_ );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << r_ct );

    std::string p_ct = "/point_cloud_merger/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( p_ct, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << p_ct );
  }

  ~PointCloudMerger () { }

private:
  ros::NodeHandle nh_;
	tf::TransformListener listener;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
	pcl::PointCloud<PointT>::Ptr scene_cloud_total;
  std::string cloud_topic_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
};

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "pcl_merger" );
	PointCloudMerger pcm;
	// Use 4 threads
	ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
