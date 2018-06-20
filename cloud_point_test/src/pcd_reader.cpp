#include <cstdlib>
#include <iostream>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_pub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1000);
  ros::Rate loop_rate(1);

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::PCDReader reader;
  reader.read ("/home/syn/ros_ws/src/cloud_point_test/PCD/Modell11122017_1030.pcd", *cloud);
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  PointCloud::Ptr msg (new PointCloud);
  pcl::fromPCLPointCloud2 (*cloud_filtered, *msg);
  for (size_t i = 0; i < msg->points.size (); ++i)
  {
    msg->points[i].x = msg->points[i].x / 1000.0;
    msg->points[i].y = msg->points[i].y / 1000.0;
    msg->points[i].z = msg->points[i].z / 1000.0;
  }
  msg->header.frame_id = "world";
  std::cout << "Loaded " << msg->width * msg->height << " data points from Modell11122017_1030.pcd " << std::endl;

  while (nh.ok())
  {
    pcl_conversions::toPCL (ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}
