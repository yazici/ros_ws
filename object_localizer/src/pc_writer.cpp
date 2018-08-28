#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ctime>

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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud< PointT > PointCloudT;
ros::Time sample_time;
static int pc_counter = 0;

class PointCloudWriter
{
public:

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    // if input cloud is empty or is not dense, publish the old point cloud and return
    if ( ( cloud->width * cloud->height ) == 0 && ! cloud->is_dense )
		{
			return;
		}

    // convert input ros cloud to pcl cloud
    // and save it in local variable scene_cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2 ( pcl_pc2, *scene_cloud_ );
		sample_time = cloud->header.stamp;
		std::cout << "[" << sample_time << "] Input point cloud has [" << scene_cloud_->width << "*" << scene_cloud_->height << " = " << scene_cloud_->width * scene_cloud_->height << "] data points" << std::endl;

		std::cout << "Save the point cloud?" << std::endl;
		char answer;
		std::cin >> answer;
		if ( answer == 'Y')
		{
      std::string pc_file_path = "/home/syn/ros_ws/src/object_localizer/pc/pc_out_" + std::to_string ( pc_counter ) + ".ply";
			std::cout << "Saving point cloud to file: \n\t" << pc_file_path << std::endl;
			writer.write ( pc_file_path, *scene_cloud_ );
			pc_counter++;
		}
  }

  PointCloudWriter () : scene_cloud_ ( new pcl::PointCloud< PointT > ) , cloud_topic_ ( "/profile_merger/points" )
  {
    cloud_sub_ = nh_.subscribe ( cloud_topic_, 30, &PointCloudWriter::cloud_cb, this );
    std::string r_ct = nh_.resolveName ( cloud_topic_ );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << r_ct );
  }

  ~PointCloudWriter () { }

private:

  ros::NodeHandle nh_;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
  std::string cloud_topic_;
  ros::Subscriber cloud_sub_;
  pcl::PLYWriter writer;
};

void CfgFileReader ()
{
  std::string pack_path = ros::package::getPath ( "object_localizer" );
  std::cout << "***The path for package [object_localizer] is: [" << pack_path << "]" << std::endl;
  std::string cfgFileName = pack_path + "/config/pc_writer.cfg";
  std::cout << "***The path of the pc_writer configuration file is: [" << cfgFileName << "]" << std::endl;

  std::ifstream input ( cfgFileName );
  std::string line;
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    std::string cmd;
    iss >> pc_counter;
    std::cout << "***pc_counter = [" << pc_counter << "]" << std::endl;
  }
  input.close();
}

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "pc_writer" );
  ros::NodeHandle n;
  CfgFileReader ();
	PointCloudWriter pcw;
	ros::spin ();
	ros::shutdown ();
  return 0;
}
