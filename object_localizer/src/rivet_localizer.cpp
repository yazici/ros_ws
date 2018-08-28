#include <stdio.h>
#include <iostream>
#include <string>

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
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

std::string SceneFileName;
int filter_mean_k = 40;
float filter_stddev = 1.0;
float scale_factor = 1.0;

// function used to show the point cloud and surface normal
void Visualize ( PointCloudT::Ptr cloud_in_transformed, PointCloudT::Ptr planar_cloud, PointCloudT::Ptr cloud_rivet ) // pcl::PointCloud< pcl::Normal >::Ptr normals
{
	pcl::visualization::PCLVisualizer viewer ( "Point Cloud Viewer" );
	int v1 ( 0 );
	viewer.createViewPort ( 0.0, 0.0, 1.0, 1.0, v1 );

	// add the point cloud to the viewer, can be updated by [updatePointCloud()]
	pcl::visualization::PointCloudColorHandlerRGBField< PointT > cloud_color_i ( cloud_in_transformed );
	viewer.addPointCloud ( cloud_in_transformed, cloud_color_i, "scene_cloud", v1 );
	viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud" );
	pcl::visualization::PointCloudColorHandlerRGBField< PointT > cloud_color_i_1 ( planar_cloud );
	viewer.addPointCloud ( planar_cloud, cloud_color_i_1, "planar_cloud", v1 );
	viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "planar_cloud" );
	pcl::visualization::PointCloudColorHandlerRGBField< PointT > cloud_color_i_2 ( cloud_rivet );
	viewer.addPointCloud ( cloud_rivet, cloud_color_i_2, "cloud_rivet", v1 );
	viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_rivet" );

	// Add normals
	// viewer.addPointCloudNormals < PointT, pcl::Normal > ( cloud, normals, 10, 0.003, "normals" );

	// Set background color
	viewer.setBackgroundColor ( 0.0, 0.0, 0.0, v1 );
	viewer.addCoordinateSystem ( 0.1 );

	// Set camera position and orientation
	// viewer.setCameraPosition( -0.0611749, -0.040113, 0.00667606, -0.105521, 0.0891437, 0.990413 );
  // Visualiser window size
  viewer.setSize ( 1280, 1024 );

	// Display the viewer
	while ( !viewer.wasStopped () )
  {
    viewer.spinOnce ();
	}

	// close the viewer
	viewer.close();
}

// downsampling an input point cloud
void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
  static pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.0001f, 0.0001f, 0.0001f );
  grid.filter ( *cloud_sampled );
	std::printf( "Downsampled cloud size is %d, %d\n", cloud_sampled->width, cloud_sampled->height );
}

// filtering an input point cloud
void filterOutliner ( PointCloudT::Ptr cloud )
{
	static pcl::StatisticalOutlierRemoval < PointT > sor;
  sor.setInputCloud ( cloud );
  sor.setMeanK ( filter_mean_k );
  sor.setStddevMulThresh ( filter_stddev );
  sor.filter ( *cloud );
}

void calculate_transform ( PointCloudT::Ptr cloud_in,  Eigen::Matrix4f& projectionTransform )
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid ( *cloud_in, pcaCentroid );
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized ( *cloud_in, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver ( covariance, Eigen::ComputeEigenvectors );
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors ();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  // std::cout << "eigen vector 0: [" << eigenVectorsPCA(0, 0) << ", " << eigenVectorsPCA(1, 0) << ", " << eigenVectorsPCA(2, 0) << "]" << std::endl;
  // std::cout << "eigen vector 1: [" << eigenVectorsPCA(0, 1) << ", " << eigenVectorsPCA(1, 1) << ", " << eigenVectorsPCA(2, 1) << "]" << std::endl;
  // std::cout << "eigen vector 2: [" << eigenVectorsPCA(0, 2) << ", " << eigenVectorsPCA(1, 2) << ", " << eigenVectorsPCA(2, 2) << "]" << std::endl;

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  // Eigen::Matrix4f projectionTransform( Eigen::Matrix4f::Identity() );
  projectionTransform.block< 3, 3 >( 0, 0 ) = eigenVectorsPCA.transpose();
  projectionTransform.block< 3, 1 >( 0, 3 ) = -1.0f * ( projectionTransform.block< 3, 3 >( 0, 0 ) * pcaCentroid.head< 3 >() );
  // pcl::transformPointCloud( *cloud_in, *cloud_out, projectionTransform );
}

void scale_and_color_point_cloud ( PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out )
{
	cloud_out->points.clear();
	int cloud_out_counter = 0;
	uint8_t r = 255, g = 255, b = 255;
	uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );

  for ( PointT temp_point: cloud_in->points )
  {
    float x = temp_point.x * scale_factor;
    float y = temp_point.y * scale_factor;
    float z = temp_point.z * scale_factor;
    PointT new_point;
    new_point.x = x;
    new_point.y = y;
    new_point.z = z;
    new_point.rgb = *reinterpret_cast<float*> ( &rgb );
    cloud_out->points.push_back( new_point );
    cloud_out_counter++;
  }
  cloud_out->width = cloud_out_counter;
  cloud_out->height = 1;
  cloud_out->header.frame_id = "world";
}

void getMinMax3D (const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt)
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

// function for finding planars
int find_planar ( PointCloudT::Ptr cloud_in )
{
	// filter, downsampling, and scaling the input point cloud and change the color of each point
	PointCloudT::Ptr cloud_filtered	( new PointCloudT );
	PointCloudT::Ptr segment_cloud ( new PointCloudT );
  filterOutliner ( cloud_in );
	downSampling ( cloud_in, cloud_filtered );
	scale_and_color_point_cloud ( cloud_filtered, segment_cloud );

	// transform the input point cloud
	PointCloudT::Ptr segment_cloud_transformed ( new PointCloudT );
	Eigen::Matrix4f transform_1 ( Eigen::Matrix4f::Identity() );
	calculate_transform ( segment_cloud, transform_1 );
	pcl::transformPointCloud( *segment_cloud, *segment_cloud_transformed, transform_1 );

	// compute surface normals
	pcl::PointCloud < pcl::Normal >::Ptr cloud_normals ( new pcl::PointCloud< pcl::Normal > );
	// std::printf("Computing surface normals...\n");
  // pcl::NormalEstimation < PointT, pcl::Normal > ne;
  // ne.setInputCloud ( segment_cloud );
  // pcl::search::KdTree < PointT >::Ptr tree ( new pcl::search::KdTree < PointT > () );
  // ne.setSearchMethod ( tree );
  // // Use neighbors in a sphere of radius 0.5mm * scale_factor
  // ne.setRadiusSearch ( 0.0005 * scale_factor );
  // ne.compute ( *cloud_normals );

	// find the planar
	pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
  pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
  pcl::SACSegmentation < PointT > seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType ( pcl::SACMODEL_PLANE );
  seg.setMethodType ( pcl::SAC_RANSAC );
  seg.setDistanceThreshold ( 0.0005 );
  seg.setInputCloud ( segment_cloud_transformed );
  seg.segment ( *inliers, *coefficients );
  if ( inliers->indices.size () == 0 )
  {
    PCL_ERROR ( "Could not estimate a planar model for the given dataset." );
    return ( -1 );
  }
	float planar_coef [3] = { coefficients->values [0], coefficients->values [1], coefficients->values [2] };
	float d_coef = coefficients->values[3];
  std::cerr << "Planar coefficients: " << planar_coef [0] << " " << planar_coef [1] << " " << planar_coef [2] << " " << d_coef << std::endl;
  std::cerr << "Planar inliers: " << inliers->indices.size () << std::endl;
	uint8_t r = 255, g = 0, b = 0;
	uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
	PointCloudT::Ptr planar_cloud	( new PointCloudT );
	int planar_cloud_counter = 0;
  for ( size_t i = 0; i < inliers->indices.size (); ++i )
	{
		segment_cloud_transformed->points[ inliers->indices[ i ] ].rgb = *reinterpret_cast<float*>( &rgb );
		PointT new_point;
    new_point.x = segment_cloud_transformed->points[ inliers->indices[ i ] ].x;
    new_point.y = segment_cloud_transformed->points[ inliers->indices[ i ] ].y;
    new_point.z = segment_cloud_transformed->points[ inliers->indices[ i ] ].z;
    new_point.rgb = *reinterpret_cast<float*>( &rgb );
    planar_cloud->points.push_back( new_point );
    planar_cloud_counter++;
  }
  planar_cloud->width = planar_cloud_counter;
  planar_cloud->height = 1;
  planar_cloud->header.frame_id = "world";
	// do transformation to show the point cloud
	PointCloudT::Ptr cloud_transformed	( new PointCloudT );
	Eigen::Matrix4f transform_2 ( Eigen::Matrix4f::Identity() );
	calculate_transform ( planar_cloud, transform_2 );
	pcl::transformPointCloud( *segment_cloud_transformed, *cloud_transformed, transform_2 );
	filterOutliner ( planar_cloud );
	pcl::transformPointCloud( *planar_cloud, *planar_cloud, transform_2 );
	pcl::PointXYZRGB minPoint, maxPoint;
  getMinMax3D( *planar_cloud, minPoint, maxPoint );
	std::cout << "minPoint = " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << std::endl;
	std::cout << "maxPoint = " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << std::endl;

	// get the total transformation
	PointCloudT::Ptr cloud_in_transformed	( new PointCloudT );
	Eigen::Matrix4f transform_total = transform_2 * transform_1;
	std::cout << "transform_2 = \n" << transform_2 << std::endl;
	std::cout << "transform_1 = \n" << transform_1 << std::endl;
	std::cout << "transform_total = \n" << transform_total << std::endl;
	scale_and_color_point_cloud ( cloud_in, cloud_in_transformed );
	pcl::transformPointCloud( *cloud_in_transformed, *cloud_in_transformed, transform_total );

	float planar_coef_abs [3] = { std::abs( planar_coef [0] ), std::abs( planar_coef [1] ), std::abs( planar_coef [2] ) };
	int max_idx = std::distance ( planar_coef_abs, std::max_element ( planar_coef_abs, planar_coef_abs + 3 ) );
	std::cout << "***Max_idx = " << max_idx << std::endl;
	// set the color of rivet
	r = 0, g = 255, b = 0;
	rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
	PointCloudT::Ptr cloud_rivet ( new PointCloudT );
	int cloud_rivet_counter = 0;
	if ( max_idx == 0 )
	{
		for ( PointT temp_point: cloud_in_transformed->points )
	  {
			float x = temp_point.x;
			float y = temp_point.y;
			float z = temp_point.z;
			if ( y >= minPoint.y && y <= maxPoint.y && z >= minPoint.z && z <= maxPoint.z && ( x >= maxPoint.x || x <= minPoint.x ) )
			{
		    PointT new_point;
		    new_point.x = x;
		    new_point.y = y;
		    new_point.z = z;
		    new_point.rgb = *reinterpret_cast<float*> ( &rgb );
		    cloud_rivet->points.push_back( new_point );
		    cloud_rivet_counter++;
			}
	  }
	}
	cloud_rivet->width = cloud_rivet_counter;
  cloud_rivet->height = 1;
  cloud_rivet->header.frame_id = "world";

	// show the point cloud
  std::printf ( "Visualizing point clouds...\n" );
	Visualize ( cloud_in_transformed, planar_cloud, cloud_rivet );
}

class RivetLocalizer
{
public:

  void handle_scene_point_cloud ( )
  {
    // load saved scene point cloud
    std::string SceneFilePath = ros::package::getPath ( "object_localizer" ) + "/data/" + SceneFileName;
    if ( pcl::io::loadPLYFile ( SceneFilePath, *scene_cloud_ ) == -1 )
    {
      ROS_INFO_STREAM ( "Couldn't read file: " << SceneFilePath << std::endl );
      return;
    }
    std::cout << "Loaded " << scene_cloud_->width * scene_cloud_->height << " data points from " << SceneFilePath << std::endl;
    find_planar ( scene_cloud_ );
  }

  RivetLocalizer () : scene_cloud_ ( new pcl::PointCloud< PointT > )
  {
  }

  ~RivetLocalizer () { }

private:
  ros::NodeHandle nh_;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
};

void CfgFileReader ()
{
  std::string pack_path = ros::package::getPath ( "object_localizer" );
  std::cout << "***The path for package [object_localizer] is: [" << pack_path << "]" << std::endl;
  std::string cfgFileName = pack_path + "/config/rivet_localizer.cfg";
  std::cout << "***The path of the rivet_localizer configuration file is: [" << cfgFileName << "]" << std::endl;

  std::ifstream input ( cfgFileName );
  std::string line;
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> SceneFileName;
    std::cout << "***SceneFileName = [" << SceneFileName << "]" << std::endl;
  }
	if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> filter_mean_k >> filter_stddev;
    std::cout << "***filter_mean_k = [" << filter_mean_k << "] filter_stddev = [" << filter_stddev << "]" << std::endl;
  }
  input.close();
}

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "rivet_localizer" );
	CfgFileReader ();
	RivetLocalizer rl;
  rl.handle_scene_point_cloud ();
	ros::spin ();
	ros::shutdown ();
  return 0;
}
