#include <cstdlib>
#include <iostream>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_pub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1000);

  PointCloud::Ptr msg (new PointCloud);
  ros::Rate loop_rate(1);

  while (nh.ok())
  {
    msg->clear();
    msg->header.frame_id = "world";
    msg->width = 100;
    msg->height = 1;
    for (int idx = 0; idx < 100; idx++)
    {
      double x = ((double) std::rand() / RAND_MAX);
      double y = ((double) std::rand() / RAND_MAX);
      double z = ((double) std::rand() / RAND_MAX);
      msg->points.push_back (pcl::PointXYZ(x, y, z));
      //std::cout << idx << " x = " << x << " y = " << y << " z = " << z << std::endl;
    }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}
