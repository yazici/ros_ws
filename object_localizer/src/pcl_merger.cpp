#include <stdio.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;
std::string camera_frame = "camera_depth_optical_frame";
ros::Time sample_time;

// function used to show the merged point cloud and calculated surface normal
void Visualize ( PointCloudT::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals )
{

	pcl::visualization::PCLVisualizer viewer ( "Merged Point Clouds" );
	int v1(0);
	viewer.createViewPort ( 0.0, 0.0, 1.0, 1.0, v1 );

	// The color we will be using
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// ICP aligned point cloud
	pcl::visualization::PointCloudColorHandlerRGBField< PointT > cloud_in_color_i ( cloud );
	viewer.addPointCloud ( cloud, cloud_in_color_i, "cloud_icp_v1", v1 );
	viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp_v1" );
	//Add normals
	viewer.addPointCloudNormals < pcl::PointXYZRGB, pcl::Normal > ( cloud, normals, 10, 0.01, "normals" );
	// Set background color
	viewer.setBackgroundColor( bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1 );
	// Set camera position and orientation
	viewer.setCameraPosition( -0.0611749, -0.040113, 0.00667606, -0.105521, 0.0891437, 0.990413 );
	viewer.setSize(1280, 1024); // Visualiser window size

	// Display the visualiser
	while ( !viewer.wasStopped () )
  {
    viewer.spinOnce ();
	}
}

void filterOutliner ( PointCloudT::Ptr cloud )
{
	//filtering point cloud
	static pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setStddevMulThresh (1.0);
  sor.setInputCloud ( cloud );
  sor.setMeanK ( cloud->size()/2 );
  sor.filter ( *cloud );
}

void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
	// std::printf( "Downsampling point clouds...\n" );
  static pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.005f, 0.005f, 0.005f );
  grid.filter ( *cloud_sampled );
	std::printf( "Downsampled cloud size is %d, %d\n", cloud_sampled->width, cloud_sampled->height );
}

// function for merging point cloud
int MergePointclouds ( PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_icp )
{
	// filter to remove outliers
	// filterOutliner ( cloud_in );
	// filterOutliner ( cloud_icp );

  // The Iterative Closest Point algorithm
  int iterations = 20;
	pcl::IterativeClosestPoint< PointT, PointT > icp;
	icp.setMaximumIterations( iterations );
	//using ICP algorithm to merge two point clouds
	std::printf( "Applying ICP..." );
	icp.setInputSource( cloud_in );
	icp.setInputTarget( cloud_icp );
	icp.align( *cloud_in );

  if ( icp.hasConverged() )
  {
    std::printf( "\nICP has converged, score is %+.0e\n", icp.getFitnessScore() );
	} else
  {
		PCL_ERROR( "\nICP has not converged.\n" );
		return -1;
	}

  // put to two point clouds together
	*cloud_icp += *cloud_in;

	//all files are processed, ICP is done
	std::printf( "ICP finishe. Merged point cloud size is %lu\n", cloud_icp->size() );

	// save data
	/*
  std::printf("Saving data to file...\n");
	pcl::PLYWriter writer;
	writer.write ("../cloud_merged.ply", *cloud_icp);
	*/

	/*
	// downsampling with voxel grid
	PointCloudT::Ptr cloud_filtered	(new PointCloudT);
	downSampling ( cloud_icp, cloud_filtered );

	//compute surface normals
	std::printf("Computing surface normals...\n");
	// Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation< pcl::PointXYZRGB, pcl::Normal > ne;
  ne.setInputCloud ( cloud_filtered );

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree< pcl::PointXYZRGB >::Ptr tree ( new pcl::search::KdTree< pcl::PointXYZRGB > () );
  ne.setSearchMethod ( tree );
  // Output datasets
  pcl::PointCloud< pcl::Normal >::Ptr cloud_normals ( new pcl::PointCloud< pcl::Normal > );

  // Use all neighbors in a sphere of radius 1cm
  ne.setRadiusSearch (0.01);

  // Compute the features
  ne.compute ( *cloud_normals );

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
	std::printf( "cloud size: %lu; normals size: %lu\n", cloud_filtered->points.size(), cloud_normals->points.size() );
  std::printf( "Visualizing point clouds...\n" );
	Visualize( cloud_filtered, cloud_normals );
	*/
}

void pointCloud_Merge_test ()
{
	// Define original point cloud and load the ply file to original point cloud
  PointCloudT::Ptr cloud_in 	(new PointCloudT);
  if ( pcl::io::loadPLYFile ( "/home/syn/ros_ws/src/object_localizer/data/octopus1.ply", *cloud_in ) == -1)
  {
    PCL_ERROR ( "Couldn't read file octopus1.pcd \n" );
    return;
  }
  std::cout << "Loaded " << cloud_in->width * cloud_in->height << " data points from octopus1.ply" << std::endl;

	// Define ICP output point cloud and load the ply file to ICP output point cloud
	PointCloudT::Ptr cloud_icp	(new PointCloudT);
  if ( pcl::io::loadPLYFile ( "/home/syn/ros_ws/src/object_localizer/data/octopus2.ply", *cloud_icp ) == -1)
  {
    PCL_ERROR ( "Couldn't read file octopus2.pcd \n" );
    return;
  }
  std::cout << "Loaded " << cloud_icp->width * cloud_icp->height << " data points from octopus2.ply" << std::endl;

	// Merge original point cloud to output point cloud
  MergePointclouds( cloud_in, cloud_icp );
}

void transform_point_cloud ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_out )
{
	static tf::TransformListener listener( ros::Duration(20), true );
  tf::StampedTransform transform;
	try
	{
		// ros::Time(0) or sample_time
    listener.lookupTransform( "world", camera_frame, sample_time, transform );
		std::cout << "tf time difference is " << ros::Time::now() - sample_time << std::endl;
		tf::Vector3 point(0, 0, 0);
		tf::Vector3 point_n(0, 0, 0);
		std::cout << "input point cloud has " << cloud->size() << " points" << std::endl;
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
  catch ( tf::TransformException ex )
  {
    ROS_ERROR("%s", ex.what());
  }
}

class PointCloudMerger
{
public:

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
    cloud_sub_ = nh_.subscribe ( cloud_topic_, 30, &PointCloudMerger::cloud_cb, this );
    std::string r_ct = nh_.resolveName ( cloud_topic_ );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << r_ct );

    std::string p_ct = "/point_cloud_merger/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( p_ct, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << p_ct );
  }

  ~PointCloudMerger () { }

private:

  ros::NodeHandle nh_;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
	pcl::PointCloud<PointT>::Ptr scene_cloud_total;
  std::string cloud_topic_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
};

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "pcl_merger" );
  ros::NodeHandle n;
	PointCloudMerger pcm;
	ros::spin ();
	ros::shutdown();
  return 0;
}
