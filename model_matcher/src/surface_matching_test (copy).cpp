#include <iostream>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>

#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/surface_matching.hpp"
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

cv::Mat pc;
ppf_match_3d::PPF3DDetector detector ( 0.025, 0.05 );

cv::Mat pointNormalsToMat ( const pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud_ptr )
{
  cv::Mat OpenCVPointCloud( point_cloud_ptr->points.size(), 6, CV_32FC1 );
  for ( int i=0; i < point_cloud_ptr->points.size(); i++ )
  {
    float* data = OpenCVPointCloud.ptr<float>(i);
    data[0] = point_cloud_ptr->points.at(i).x;
    data[1] = point_cloud_ptr->points.at(i).y;
    data[2] = point_cloud_ptr->points.at(i).z;
    float n_x, n_y, n_z;
    n_x = point_cloud_ptr->points.at(i).normal[0];
    n_y = point_cloud_ptr->points.at(i).normal[1];
    n_z = point_cloud_ptr->points.at(i).normal[2];
    float norm = sqrt(n_x*n_x + n_y*n_y + n_z*n_z);
    if ( norm > 0.00001 )
    {
      data[3] = n_x / static_cast<float>(norm);
      data[4] = n_y / static_cast<float>(norm);
      data[5] = n_z / static_cast<float>(norm);
    }
    else
    {
      data[3] = n_x;
      data[4] = n_y;
      data[5] = n_z;
    }
  }
  return OpenCVPointCloud;
}

void getPointNormals ( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_point_normals )
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.05);
  // Compute point normals
  ne.compute (*cloud_normals);
  pcl::concatenateFields (*cloud, *cloud_normals, *cloud_point_normals);
}

void loadAndTrainObjectModel ()
{
  // load the object model file
  string modelFileName = "/home/syn/ros_ws/src/model_matcher/model/platte_mit_schrauben.ply";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> );
  if ( pcl::io::loadPLYFile<pcl::PointXYZ> (modelFileName, *cloud) == -1 )
  {
    PCL_ERROR ("Couldn't read file platte_mit_schrauben.ply \n");
  }
  // change the scale of input data
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = cloud->points[i].x / 1000.0;
    cloud->points[i].y = cloud->points[i].y / 1000.0;
    cloud->points[i].z = cloud->points[i].z / 1000.0;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals(new pcl::PointCloud<pcl::PointNormal>);
  getPointNormals ( cloud, cloud_point_normals );
  pc = pointNormalsToMat ( cloud_point_normals );

  // Start training with the object model
  std::cout << "Start Training the object model..." << endl;
  int64 tick1 = cv::getTickCount();
  detector.trainModel(pc);
  int64 tick2 = cv::getTickCount();
  cout << "Training complete in " << (float)(tick2-tick1) / cv::getTickFrequency() << " sec" << endl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pcl::PCLPointCloud2::Ptr cloud_PC2 (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered_PC2 (new pcl::PCLPointCloud2 ());
  pcl::PCDReader reader;
  reader.read ("/home/syn/ros_ws/src/cloud_point_test/PCD/Modell11122017_1030.pcd", *cloud_PC2);
  std::cerr << "PointCloud before filtering: " << cloud_PC2->width * cloud_PC2->height << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_PC2);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
  sor.filter (*cloud_filtered_PC2);

  std::cerr << "PointCloud after filtering: " << cloud_filtered_PC2->width * cloud_filtered_PC2->height << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2 (*cloud_filtered_PC2, *msg);
  for (size_t i = 0; i < msg->points.size (); ++i)
  {
    msg->points[i].x = msg->points[i].x / 1000.0;
    msg->points[i].y = msg->points[i].y / 1000.0;
    msg->points[i].z = msg->points[i].z / 1000.0;
  }
  msg->header.frame_id = "world";
  std::cout << "Loaded " << msg->width * msg->height << " data points from Modell11122017_1030.pcd " << std::endl;
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("points2", 1000);
  pcl_conversions::toPCL (ros::Time::now(), msg->header.stamp);
  ros::Rate loop_rate(20);
  int counter = 0;
  while (counter < 100)
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
    counter++;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals2(new pcl::PointCloud<pcl::PointNormal>);
  getPointNormals ( msg, cloud_point_normals2 );
  cv::Mat pcTest = pointNormalsToMat ( cloud_point_normals2 );
  std::cout << "[scene mat rows, cols]: " << pcTest.rows << ", " << pcTest.cols << std::endl;
  /*
  for (int i = 0; i < pcTest.rows; i++)
  {
    std::cout << i << ": ";
    for (int col = 0; col < pcTest.cols; ++col)
    {
       std::cout << pcTest.at<float>(i, col) << " ";
    }
    std::cout << std::endl;
  }
  //*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << endl << "Starting matching..." << endl;
  vector<Pose3DPtr> results;
  tick1 = cv::getTickCount();
  detector.match(pcTest, results, 1.0/40.0, 0.05);
  tick2 = cv::getTickCount();
  cout << endl << "PPF Elapsed Time " << (float)(tick2 - tick1) / cv::getTickFrequency() << " sec" << endl;

  // Get only first N results
  int N = 2;
  vector<Pose3DPtr> resultsSub( results.begin(), results.begin()+N );

  // Create an instance of ICP
  ICP icp(100, 0.005f, 2.5f, 8);
  int64 t1 = cv::getTickCount();
  // Register for all selected poses
  cout << endl << "Performing ICP on " << N << " poses..." << endl;
  icp.registerModelToScene(pc, pcTest, resultsSub);
  int64 t2 = cv::getTickCount();

  cout << endl << "ICP Elapsed Time " << (float)(t2 - t1) / cv::getTickFrequency() << " sec" << endl;

  cout << "Poses: " << endl;
  for ( size_t i = 0; i < resultsSub.size(); i++ )
  {
      Pose3DPtr result = resultsSub[i];
      std::cout << "Pose Result [x, y, z] = [" << result->t[0] << " " << result->t[1] << " " << result->t[2] << "], Rotation = ["
                << result->q[0] << " " << result->q[1] << " " << result->q[2] << " " << result->q[3] << "]" << std::endl;
  }

}

void pointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr& cloud_msg )
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL( *cloud_msg, pcl_pc2 );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud< pcl::PointXYZ > );
  pcl::fromPCLPointCloud2( pcl_pc2, *cloud );
  std::cout << "[scene cloud width, height]:" << cloud->width << ", " << cloud->height << std::endl;

  /*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals(new pcl::PointCloud<pcl::PointNormal>);
  getPointNormals ( cloud, cloud_point_normals );
  cv::Mat pcTest = pointNormalsToMat ( cloud_point_normals );
  std::cout << "[scene mat rows, cols]: " << pcTest.rows << ", " << pcTest.cols << std::endl;
  /*
  for (int i = 0; i < pcTest.rows; i++)
  {
    std::cout << i << ": ";
    for (int col = 0; col < pcTest.cols; ++col)
    {
       std::cout << pcTest.at<float>(i, col) << " ";
    }
    std::cout << std::endl;
  }

  cout << endl << "Starting matching..." << endl;
  vector<Pose3DPtr> results;
  int64 tick1 = cv::getTickCount();
  detector.match(pcTest, results, 1.0/40.0, 0.05);
  int64 tick2 = cv::getTickCount();
  cout << endl << "PPF Elapsed Time " << (float)(tick2 - tick1) / cv::getTickFrequency() << " sec" << endl;

  // Get only first N results
  int N = 2;
  vector<Pose3DPtr> resultsSub( results.begin(), results.begin()+N );

  // Create an instance of ICP
  ICP icp(100, 0.005f, 2.5f, 8);
  int64 t1 = cv::getTickCount();
  // Register for all selected poses
  cout << endl << "Performing ICP on " << N << " poses..." << endl;
  icp.registerModelToScene(pc, pcTest, resultsSub);
  int64 t2 = cv::getTickCount();

  cout << endl << "ICP Elapsed Time " << (float)(t2 - t1) / cv::getTickFrequency() << " sec" << endl;

  cout << "Poses: " << endl;
  for ( size_t i = 0; i < resultsSub.size(); i++ )
  {
      Pose3DPtr result = resultsSub[i];
      cout << "Pose Result " << i << endl;
      result->printPose();
  }
  */
}

int main ( int argc, char** argv )
{
  ros::init(argc, argv, "ARObject");
  ros::NodeHandle node;

  // preprocess the 3D model
  loadAndTrainObjectModel();

  // subscribe to the input point cloud topic
  ros::Subscriber sub = node.subscribe( "/pico_flexx/points", 100, pointCloudCallback );
  //ros::AsyncSpinner spinner(4);
  //spinner.start();
  ros::spin();

  return 0;
}
