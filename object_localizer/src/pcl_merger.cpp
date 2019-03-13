#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>
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
std::string reference_frame = "world";
std::string camera_frame = "camera_color_optical_frame";
ros::Time sample_time;

void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
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
		    listener.lookupTransform( reference_frame, camera_frame, sample_time, transform );
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
			cloud_out->points.push_back ( temp_point );
		}
	}

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    // if don't merge or input cloud is not dense or is empty, publish the old point cloud and return
    // !cloud->is_dense the is dense doesn't work any more.
    if ( !is_merge_ || ( cloud->width * cloud->height ) == 0 )
		{
      scene_cloud_total->header.frame_id = reference_frame;
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
		scene_cloud_total->header.frame_id = reference_frame;
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

  bool start_pcl_merge ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    // clear old merger point cloud
    scene_cloud_total->clear ();
    scene_cloud_total->header.frame_id = reference_frame;
    pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_total->header.stamp );
    cloud_pub_.publish ( scene_cloud_total );
    // then start to merge new point cloud
    is_merge_ = true;
    return true;
  }

  bool end_pcl_merge ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    is_merge_ = false;
    // clear the scene_cloud_total point cloud and publish the empty one
    // std::string pcl_file_path = ros::package::getPath ( "object_localizer" )+ "/data/pcl_out_0.ply";
    // std::cout << "Saving point cloud to file: \n\t" << pcl_file_path << std::endl;
    // writer.write ( pcl_file_path, *scene_cloud_total );
    return true;
  }

  PointCloudMerger () : scene_cloud_ ( new pcl::PointCloud< PointT > ), scene_cloud_total ( new pcl::PointCloud< PointT > )
  {
    is_merge_ = false;
    start_pcl_merge_ = nh_.advertiseService ( "start_pcl_merge", &PointCloudMerger::start_pcl_merge, this );
    end_pcl_merge_ = nh_.advertiseService ( "stop_pcl_merge", &PointCloudMerger::end_pcl_merge, this );
    ros::Duration ( 1 ).sleep ();

    std::string cloud_topic_in_ = "/camera/depth_registered/points";
    cloud_sub_ = nh_.subscribe ( cloud_topic_in_, 200, &PointCloudMerger::cloud_cb, this );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << cloud_topic_in_ );

    std::string cloud_topic_out_ = "/point_cloud_merger/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_topic_out_, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_topic_out_ );
  }

  ~PointCloudMerger () { }

private:
  ros::NodeHandle nh_;
	tf::TransformListener listener;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
	pcl::PointCloud<PointT>::Ptr scene_cloud_total;
  bool is_merge_;
  ros::ServiceServer start_pcl_merge_, end_pcl_merge_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
  // pcl::PLYWriter writer;
};

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "pcl_merger" );
	ros::AsyncSpinner spinner(4);
  spinner.start();
  PointCloudMerger pcm;
  ros::waitForShutdown();
  return 0;
}
