#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

// converse header file between ros points2 and pcl point cloud
#include <pcl_conversions/pcl_conversions.h>
#include "object_localizer_msg/BBox_int.h"
#include "object_localizer_msg/BBox_float.h"
#include "object_localizer_msg/BBox_list.h"
#include "object_localizer_msg/Segment_list.h"

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
#include <pcl/common/transforms.h>

std::string reference_frame = "world";

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

float scan_offset = 0.08;
float scan_distance = 0.08;

void show_segment_cloud ( PointCloudT::ConstPtr cloud )
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer ( new pcl::visualization::PCLVisualizer ( "Segment Viewer" ) );
  viewer->setBackgroundColor ( 0, 0, 0 );
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb( cloud );
  viewer->addPointCloud< PointT > ( cloud, rgb, "segment cloud" );
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segment cloud" );
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  while ( !viewer->wasStopped () )
  {
    viewer->spinOnce ( 100 );
    boost::this_thread::sleep ( boost::posix_time::microseconds (100000) );
  }
}

void getMinMax3D ( const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt )
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) ||
          !pcl_isfinite (cloud.points[i].y) ||
          !pcl_isfinite (cloud.points[i].z))
          continue;
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  min_pt.x = min_p[0]; min_pt.y = min_p[1]; min_pt.z = min_p[2];
  max_pt.x = max_p[0]; max_pt.y = max_p[1]; max_pt.z = max_p[2];
}

void calculate_bounding_box ( PointCloudT::ConstPtr cloudSegmented )
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid( *cloudSegmented, pcaCentroid );
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized( *cloudSegmented, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver( covariance, Eigen::ComputeEigenvectors );
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
  // the signs are different and the box doesn't get correctly oriented in some cases.
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  std::cout << "eigen vector 0: [" << eigenVectorsPCA(0, 0) << ", " << eigenVectorsPCA(1, 0) << ", " << eigenVectorsPCA(2, 0) << "]" << std::endl;
  std::cout << "eigen vector 1: [" << eigenVectorsPCA(0, 1) << ", " << eigenVectorsPCA(1, 1) << ", " << eigenVectorsPCA(2, 1) << "]" << std::endl;
  std::cout << "eigen vector 2: [" << eigenVectorsPCA(0, 2) << ", " << eigenVectorsPCA(1, 2) << ", " << eigenVectorsPCA(2, 2) << "]" << std::endl;
  // std::cout << "eigen vector 1: " << eigenVectorsPCA.col(1) << std::endl;
  // std::cout << "eigen vector 2: " << eigenVectorsPCA.col(2) << std::endl;

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform( Eigen::Matrix4f::Identity() );
  projectionTransform.block< 3,3 >( 0, 0 ) = eigenVectorsPCA.transpose();
  projectionTransform.block< 3,1 >( 0, 3 ) = -1.0f * ( projectionTransform.block< 3, 3 >( 0, 0 ) * pcaCentroid.head< 3 >() );
  pcl::PointCloud< PointT >::Ptr cloudPointsProjected ( new pcl::PointCloud< PointT > );
  pcl::transformPointCloud( *cloudSegmented, *cloudPointsProjected, projectionTransform );
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZRGB minPoint, maxPoint;
  getMinMax3D( *cloudPointsProjected, minPoint, maxPoint );
  const Eigen::Vector3f meanDiagonal = 0.5f * ( maxPoint.getVector3fMap() + minPoint.getVector3fMap() );

  // calculate quaternion from eigenvectors
  const Eigen::Quaternionf bboxQuaternion( eigenVectorsPCA );
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head< 3 >();

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer ( new pcl::visualization::PCLVisualizer ( "Segment Viewer" ) );
  viewer->setBackgroundColor ( 0, 0, 0 );
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb( cloudSegmented );
  viewer->addPointCloud< PointT > ( cloudSegmented, rgb, "segment cloud" );
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segment cloud" );
  viewer->addCoordinateSystem ( 1.0 );
  viewer->addCube ( bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox" );
  viewer->initCameraParameters ();
  while ( !viewer->wasStopped () )
  {
    viewer->spinOnce ( 100 );
    boost::this_thread::sleep ( boost::posix_time::microseconds (100000) );
  }
}

template < typename T > int sgn ( T val )
{
    return (T(0) < val) - (val < T(0));
}

float calculate_theta ( PointCloudT::ConstPtr cloudSegmented, Eigen::Vector3f& central_point )
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid( *cloudSegmented, pcaCentroid );
  central_point = pcaCentroid.head< 3 >();
  // std::cout << "central point is " << central_point << std::endl;

  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized( *cloudSegmented, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver( covariance, Eigen::ComputeEigenvectors );
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
  // the signs are different and the box doesn't get correctly oriented in some cases.
  eigenVectorsPCA.col ( 2 ) = eigenVectorsPCA.col ( 0 ).cross ( eigenVectorsPCA.col ( 1 ) );

  std::cout << "eigen vector 0: [" << eigenVectorsPCA (0, 0) << ", " << eigenVectorsPCA (1, 0) << ", " << eigenVectorsPCA (2, 0) << "]" << std::endl;
  if ( std::abs( eigenVectorsPCA ( 0, 0 ) ) < 0.1 &&  sgn<float>( eigenVectorsPCA ( 1, 0 ) ) == sgn<float>( eigenVectorsPCA ( 2, 0 ) ) )
  {
    float y = std::abs( eigenVectorsPCA ( 1, 0 ) );
    float z = std::abs( eigenVectorsPCA ( 2, 0 ) );
    float theta = atan2 ( z, y ) * 180.0 / M_PI + 90.0;
    return theta;
  }

  std::cout << "eigen vector 1: [" << eigenVectorsPCA ( 0, 1 ) << ", " << eigenVectorsPCA ( 1, 1 ) << ", " << eigenVectorsPCA ( 2, 1 ) << "]" << std::endl;
  if ( std::abs( eigenVectorsPCA ( 0, 1 ) ) < 0.1 &&  sgn<float>( eigenVectorsPCA ( 1, 1 ) ) == sgn<float>( eigenVectorsPCA ( 2, 1 ) ) )
  {
    float y = std::abs( eigenVectorsPCA ( 1, 1 ) );
    float z = std::abs( eigenVectorsPCA ( 2, 1 ) );
    float theta = atan2 ( z, y ) * 180.0 / M_PI + 90.0;
    return theta;
  }

  std::cout << "eigen vector 2: [" << eigenVectorsPCA ( 0, 2 ) << ", " << eigenVectorsPCA ( 1, 2 ) << ", " << eigenVectorsPCA ( 2, 2 ) << "]" << std::endl;
  if ( std::abs( eigenVectorsPCA ( 0, 2 ) ) < 0.1 &&  sgn<float>( eigenVectorsPCA ( 1, 2 ) ) == sgn<float>( eigenVectorsPCA ( 2, 2 ) ) )
  {
    float y = std::abs( eigenVectorsPCA ( 1, 2 ) );
    float z = std::abs( eigenVectorsPCA ( 2, 2 ) );
    float theta = atan2 ( z, y ) * 180.0 / M_PI + 90.0;
    return theta;
  }
}

class ScanPlanner
{
public:

  void segment_list_cb ( const object_localizer_msg::Segment_list::ConstPtr& segment_list_in )
  {
    if ( segment_list_in->BBox_list_float.size() > 0 )
    {
      segment_list.reset ( new object_localizer_msg::Segment_list () );
      int bbox_idx = 0;
      for ( object_localizer_msg::BBox_float bbox : segment_list_in->BBox_list_float )
      {
        segment_list->BBox_list_float.push_back ( bbox );
        segment_list->Segment_list.push_back ( segment_list_in->Segment_list [ bbox_idx ] );
        bbox_idx ++;
      }
    }
  }

  bool start_scan_planner ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    if ( segment_list->BBox_list_float.size() > 0 )
    {
      ofstream do_scan_fs;
      std::string cfgFileName = ros::package::getPath ( "motion_control" ) + "/config/do_scan.cfg";
      do_scan_fs.open ( cfgFileName );

      // step 1, show all possible segments
			int bbox_idx = 0;
      for ( object_localizer_msg::BBox_float bbox : segment_list->BBox_list_float )
      {
        // step 2, filter out bounding boxes of possible rivets
        if ( bbox.x1 > 0.15 && bbox.x2 > 0.15 )
        {
          std::cout << "Bounding box [" << bbox_idx << "] has [x1, x2, y1, y2]: [" << bbox.x1 << "," << bbox.x2 << "," << bbox.y1 << "," << bbox.y2  << "]" << std::endl;

          // step 3, generate scan plan for the chosen segment
          pcl::PCLPointCloud2 pcl_pc2;
          pcl_conversions::toPCL ( segment_list->Segment_list [ bbox_idx ], pcl_pc2 );
          PointCloudT::Ptr segment_cloud ( new PointCloudT );
          pcl::fromPCLPointCloud2 ( pcl_pc2, *segment_cloud );
          // show_segment_cloud( segment_cloud );
          // calculate_bounding_box( segment_cloud );
          Eigen::Vector3f central_point;
          float theta = calculate_theta ( segment_cloud, central_point );
          float x_0 = central_point ( 0 );
          float y_0 = central_point ( 1 );
          float z_0 = central_point ( 2 );

          float theta_tmp = ( theta - 90.0 ) * M_PI / 180.0;
          float x_tmp = x_0;
          float y_tmp = y_0 + scan_offset * std::sin ( theta_tmp );
          float z_tmp = z_0 - scan_offset * std::cos ( theta_tmp );
          float x_s = x_tmp;
          float y_s = y_tmp - scan_distance * std::cos ( theta_tmp );
          float z_s = z_tmp - scan_distance * std::sin ( theta_tmp );
          float x_e = x_tmp;
          float y_e = y_tmp + scan_distance * std::cos ( theta_tmp );
          float z_e = z_tmp + scan_distance * std::sin ( theta_tmp );

          // step 4, write scanning plannings.
          std::cout << std::endl << "[***] Rotation around x is [" << theta << "] degrees" << std::endl;
          // std::cout << "[***] Scan central point is [x, y, z] = [" << x_tmp << ", " << y_tmp << ", " << z_tmp << "]" << std::endl;
          std::cout << "[***] Scan start point is [x, y, z] = [" << x_s << ", " << y_s << ", " << z_s << "]" << std::endl;
          std::cout << "[***] Scan end point is [x, y, z] = [" << x_e << ", " << y_e << ", " << z_e << "]" << std::endl << std::endl;
          do_scan_fs << theta << " " << x_s << " " << y_s << " " << z_s << " " << x_e << " " << y_e << " " << z_e << std::endl;
        }
        bbox_idx ++;
      }
    	do_scan_fs.close();
    }
    return true;
  }

  ScanPlanner ()
  {
    start_scan_planner_ = nh_.advertiseService ( "start_scan_planner", &ScanPlanner::start_scan_planner, this );
    ros::Duration ( 0.5 ) .sleep ();

    std::string segment_list_in_name = "/box_segmenter/segment_list";
    segment_list_sub_ = nh_.subscribe ( segment_list_in_name, 10, &ScanPlanner::segment_list_cb, this );
    ROS_INFO_STREAM ( "Listening for segment list on topic: " << segment_list_in_name );

    start_profile_scan_ = nh_.serviceClient < std_srvs::Empty > ( "start_profile_scan" );
  }

  ~ScanPlanner () { }

private:
  ros::NodeHandle nh_;
  object_localizer_msg::Segment_list::Ptr segment_list;
  ros::ServiceServer start_scan_planner_;
  ros::Subscriber segment_list_sub_;
  ros::ServiceClient start_profile_scan_;
};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "scan_planner" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ScanPlanner SP;
  ros::waitForShutdown ();
  return 0;
}
