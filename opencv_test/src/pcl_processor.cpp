#include <stdio.h>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ctime>

#include <sensor_msgs/PointCloud2.h>

// converse header file between ros points2 and pcl point cloud
#include <pcl_conversions/pcl_conversions.h>
#include "opencv_test/BBox_list.h"
#include "opencv_test/BBox.h"

// PCL head files
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

std::string camera_frame = "camera_link";

class PointCloudProcessor
{

public:

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    // if input cloud is empty, return
    if ( ( cloud->width * cloud->height ) == 0 )
      return;

    // convert input ros cloud to pcl cloud
    // and save it in local variable scene_cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2( pcl_pc2, *scene_cloud_ );

    // scene_cloud_->header.frame_id = camera_frame;
    // pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_->header.stamp );
    // cloud_pub_.publish ( scene_cloud_ );
  }

  void bbox_cb ( const opencv_test::BBox_list::ConstPtr& bbox_list )
  {
    pcl::PointCloud<pcl::PointXYZ> cropped_pcloud;

    if ( bbox_list->BBox_list.size() > 0 )
    {
      for ( opencv_test::BBox bbox : bbox_list->BBox_list )
      {
        while ( scene_cloud_->width == 0 )
        {
          ros::spinOnce();
        }
        std::cout << "Scene cloud has [frame_id, width, height]: " << scene_cloud_->header.frame_id << ", " << scene_cloud_->width << ", " << scene_cloud_->height << std::endl;
        std::cout << "Bounding box has [x1, x2, y1, y2]: " << bbox.x1 << "\t" << bbox.x2 << "\t" << bbox.y1 << "\t" << bbox.y2 << std::endl;
        for ( int idx_x = bbox.x1; idx_x <= bbox.x2 && idx_x < scene_cloud_->height; idx_x++ )
        {
          for ( int idx_y = bbox.y1; idx_y <= bbox.y2 && idx_y < scene_cloud_->width; idx_y++ )
          {
            cropped_pcloud.points.push_back( scene_cloud_->at(idx_y, idx_x) );
          }
        }
      }
    }

    cropped_pcloud.header.frame_id = scene_cloud_->header.frame_id;
    pcl_conversions::toPCL ( ros::Time::now(), cropped_pcloud.header.stamp );
    cloud_pub_.publish ( cropped_pcloud );
  }

  PointCloudProcessor () : scene_cloud_ ( new pcl::PointCloud< pcl::PointXYZ > ), cloud_topic_ ( "/camera/depth_registered/points" ), bbox_topic_ ( "/object_localizer/bbox_list" )
  {
    cloud_sub_ = nh_.subscribe ( cloud_topic_, 30, &PointCloudProcessor::cloud_cb, this );
    std::string r_ct = nh_.resolveName ( cloud_topic_ );
    ROS_INFO_STREAM ( "Listening for incoming point cloud on topic " << r_ct );

    std::string p_ct = "/pcl_processor/points";
    cloud_pub_ = nh_.advertise < pcl::PointCloud < pcl::PointXYZ > > ( p_ct, 30 );
    ROS_INFO_STREAM ( "publish point cloud message on topic " << p_ct );

    bbox_sub_ = nh_.subscribe ( bbox_topic_, 30, &PointCloudProcessor::bbox_cb, this );
    std::string r_bt = nh_.resolveName ( bbox_topic_ );
    ROS_INFO_STREAM ( "Listening for bounding box list on topic " << r_bt );
  }

  ~PointCloudProcessor () { }

private:

  ros::NodeHandle nh_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_;
  std::string cloud_topic_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
  std::string bbox_topic_;
  ros::Subscriber bbox_sub_;

};

int main( int argc, char** argv )
{
  ros::init ( argc, argv, "pcl_processor" );
  ros::NodeHandle n;
  PointCloudProcessor pcp;
  ros::spin ();
  return 0;
}
