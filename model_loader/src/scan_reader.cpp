#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <tf/transform_broadcaster.h>
#include <cmath>

#include <stdio.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

#include <geometric_shapes/shape_messages.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <ros/package.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/JointState.h>

laser_geometry::LaserProjection projector_;
ros::Publisher pub;
ros::Publisher pub_joint;
int msg_counter = 0;
float point1_x = 0.0;
float point2_x = 0.0;
float point1_y = 0.0;
float point2_y = 0.0;
float aircraft_frame_y_offset = 110; //cm
float table_y_offset = 30; //cm
float table_x_offset = 0; //cm
float point_z = 0.55; //m

void get_the_central_point()
{
  std::cout << "Value of point1 is [" << point1_x << ", " << point1_y << "], point1 is [" << point2_x << ", " << point2_y << "]" << std::endl;
  float point1_x_tmp = std::round(point1_x*100.0);
  float point2_x_tmp = std::round(point2_x*100.0);
  float point1_y_tmp = std::round(point1_y*100.0);
  float point2_y_tmp = std::round(point2_y*100.0);
  float x0 = (point1_x_tmp + point2_x_tmp) / 2.0 + table_x_offset;
  float y0 = (point1_y_tmp + point2_y_tmp) / 2.0 + table_y_offset;
  float distance = std::sqrt( std::pow( point1_x_tmp -point2_x_tmp, 2 ) + std::pow( point1_y_tmp - point2_y_tmp, 2 ) );
  std::cout << "|y1 - y2| = " << std::abs(point1_y_tmp - point2_y_tmp) << " distance between p1 and p2 is:" << distance << std::endl;

  if ( std::abs(point1_y_tmp - point2_y_tmp) > 6 && std::abs( distance -88.0 ) > 2 )
  {
    std::cerr << "|y1 - y2| = " << std::abs(point1_y_tmp - point2_y_tmp) << std::endl;
    std::cerr << "###-> The robot need to face the aircraft_frame and have no people at front" << std::endl;
    return;
  }

  float x = std::round( -x0 ) / 100.0;
  float y = std::round( aircraft_frame_y_offset - y0 ) / 100.0;
  float z = point_z;
  msg_counter++;
  std::cout << msg_counter << ": [x0, y0] = [" << x0 << ", " << y0 << "]; [x, y] = [" << x << ", " << y << "]" << std::endl;

  sensor_msgs::JointState table_joint_msg;
  table_joint_msg.header.stamp = ros::Time::now();
  table_joint_msg.name.push_back("table_floor_x_joint");
  table_joint_msg.name.push_back("table_floor_y_joint");
  table_joint_msg.position.push_back(x);
  table_joint_msg.position.push_back(y);
  pub_joint.publish(table_joint_msg);

}

void scanCallback( const sensor_msgs::LaserScan::ConstPtr& scan_in )
{
  // convert from laserscan to pcl point cloud
  sensor_msgs::PointCloud2 sensor_pc2;
  projector_.projectLaser( *scan_in, sensor_pc2 );

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL( sensor_pc2, pcl_pc2 );

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud( new pcl::PointCloud< pcl::PointXYZ > );
  pcl::fromPCLPointCloud2( pcl_pc2, *temp_cloud );
  temp_cloud->header.frame_id = "scan";
  pcl_conversions::toPCL( ros::Time::now(), temp_cloud->header.stamp );
  //pub.publish( temp_cloud );

  if( temp_cloud->height == 1 )
  {
    ///*
    // filter point within the range x = [-2.2, 0.0], y = [-2.0, 0.0] and x = [0.0, 2.2], y = [-2.0, 0.0]
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_left (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> temp_vector_left;
    int point_counter_left = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_right (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> temp_vector_right;
    int point_counter_right = 0;
    for( pcl::PointXYZ temp_point : temp_cloud->points )
    {
        if( temp_point.y > -2 && temp_point.y < 0.0 )
        {
          if ( temp_point.x > -2.2 && temp_point.x < 0.0 )
          {
            temp_vector_left.push_back ( temp_point );
            point_counter_left++;
          } else if ( temp_point.x > 0.0 && temp_point.x < 2.2 )
          {
            temp_vector_right.push_back ( temp_point );
            point_counter_right++;
          }
        }
    }

    temp_cloud_left->header.frame_id = "scan";
    temp_cloud_left->width = point_counter_left;
    temp_cloud_left->height = 1;
    for( pcl::PointXYZ temp_point : temp_vector_left )
    {
      temp_cloud_left->points.push_back (temp_point);
    }
    pcl_conversions::toPCL(ros::Time::now(), temp_cloud_left->header.stamp);
    //pub.publish (temp_cloud_left);

    temp_cloud_right->header.frame_id = "scan";
    temp_cloud_right->width = point_counter_right;
    temp_cloud_right->height = 1;
    for( pcl::PointXYZ temp_point : temp_vector_right )
    {
      temp_cloud_right->points.push_back (temp_point);
    }
    pcl_conversions::toPCL(ros::Time::now(), temp_cloud_right->header.stamp);
    //pub.publish (temp_cloud_right);

    // serch for 10 nearest point
    pcl::KdTreeFLANN< pcl::PointXYZ > kdtree;
    pcl::PointXYZ searchPoint;
    searchPoint.x = 0.0;
    searchPoint.y = 0.0;
    searchPoint.z = 0.0;
    int K = 10;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::vector<pcl::PointXYZ> final_vector_left;
    kdtree.setInputCloud ( temp_cloud_left );
    if ( kdtree.nearestKSearch ( searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      float x_sum = 0.0;
      float y_sum = 0.0;
      for ( size_t i = 0; i < pointIdxNKNSearch.size (); ++i )
      {
        pcl::PointXYZ temp_point;
        temp_point.x = temp_cloud_left->points[ pointIdxNKNSearch[i] ].x;
        x_sum += temp_point.x;
        temp_point.y = temp_cloud_left->points[ pointIdxNKNSearch[i] ].y;
        y_sum += temp_point.y;
        temp_point.z = temp_cloud_left->points[ pointIdxNKNSearch[i] ].z;
        final_vector_left.push_back ( temp_point );
        //std::cout << "[" << final_vector_left[i].x << ", " << final_vector_left[i].y << ", " << final_vector_left[i].z << "]; ";
      }
      if ( point1_x != 0.0)
      {
        point1_x = ( point1_x + x_sum/10.0 ) / 2.0;
      } else
      {
        point1_x = x_sum/10.0;
      }
      if ( point1_y != 0.0)
      {
        point1_y = ( point1_y + y_sum/10.0 ) / 2.0;
      } else
      {
        point1_y = y_sum/10.0;
      }
      //std::cout << std::endl;
    }
    //std::cout << final_vector_left.size () << std::endl;

    std::vector<pcl::PointXYZ> final_vector_right;
    kdtree.setInputCloud ( temp_cloud_right );
    if ( kdtree.nearestKSearch ( searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      float x_sum = 0.0;
      float y_sum = 0.0;
      for ( size_t i = 0; i < pointIdxNKNSearch.size (); ++i )
      {
        pcl::PointXYZ temp_point;
        temp_point.x = temp_cloud_right->points[ pointIdxNKNSearch[i] ].x;
        x_sum += temp_point.x;
        temp_point.y = temp_cloud_right->points[ pointIdxNKNSearch[i] ].y;
        y_sum += temp_point.y;
        temp_point.z = temp_cloud_right->points[ pointIdxNKNSearch[i] ].z;
        final_vector_right.push_back ( temp_point );
        //std::cout << "[" << final_vector_right[i].x << ", " << final_vector_right[i].y << ", " << final_vector_right[i].z << "]; ";
      }
      if ( point2_x != 0.0)
      {
        point2_x = ( point2_x + x_sum/10.0 ) / 2.0;
      } else
      {
        point2_x = x_sum/10.0;
      }
      if ( point2_y != 0.0)
      {
        point2_y = ( point2_y + y_sum/10.0 ) / 2.0;
      } else
      {
        point2_y = y_sum/10.0;
      }
      //std::cout << std::endl;
    }
    //std::cout << final_vector_right.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_final->header.frame_id = "scan";
    cloud_final->width = final_vector_left.size () + final_vector_right.size ();
    //std::cout << cloud_final->width << std::endl;
    cloud_final->height = 1;
    for( pcl::PointXYZ temp_point : final_vector_left )
    {
      cloud_final->points.push_back (temp_point);
    }
    for( pcl::PointXYZ temp_point : final_vector_right )
    {
      cloud_final->points.push_back ( temp_point );
    }
    pcl_conversions::toPCL(ros::Time::now(), cloud_final->header.stamp);
    pub.publish( cloud_final );

  }

  //std::cout << "point1 = [" << point1_x << ", " << point1_y << "]; point2 = [" << point2_x << ", " << point2_y << "]" << std::endl;
  get_the_central_point();

}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "scan_reader" );
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe( "/r2000_driver_node/scan", 10, scanCallback );
  pub = nh.advertise< pcl::PointCloud< pcl::PointXYZ > >( "points2", 1000 );
  pub_joint = nh.advertise< sensor_msgs::JointState >( "joint_states", 100 );
  ros::spin();

  return 0;
}
