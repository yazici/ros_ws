#include <iostream>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      // scale the loaded model PCD file from millimeter to meter
      for (size_t i = 0; i < xyz_->points.size (); ++i)
      {
        xyz_->points[i].x = xyz_->points[i].x / 1000.0;
        xyz_->points[i].y = xyz_->points[i].y / 1000.0;
        xyz_->points[i].z = xyz_->points[i].z / 1000.0;
      }

      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

std::vector<FeatureCloud> object_templates;

void loadAndTrainObjectModel ()
{
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // step 1: loading object models
  string modelFileName = "/home/syn/ros_ws/src/model_matcher/model/platte_mit_schrauben.pcd";
  FeatureCloud template_cloud;
  template_cloud.loadInputCloud (modelFileName);
  object_templates.push_back (template_cloud);
}

void pointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr& cloud_msg )
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // step 2: convert input cloud point msg to scene point cloud (target point cloud)
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL( *cloud_msg, pcl_pc2 );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud< pcl::PointXYZ > );
  pcl::fromPCLPointCloud2( pcl_pc2, *cloud );
  std::cout << "Original scene cloud has [width, height]:" << cloud->width << ", " << cloud->height << std::endl;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // step 3: preprocessing the target point cloud
  // removing distant points above 1.0 meter
  const float depth_limit = 1.0;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);

  // downsampling the point cloud with grid size = 5 millimeter
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  // vox_grid.filter (*cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
  vox_grid.filter (*tempCloud);
  cloud = tempCloud;

  // show the preprocessed target point cloud
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("/surface_matching/points2", 1000);
  ros::Rate loop_rate(20);
  cloud->header.frame_id = "camera_frame";
  int pub_counter = 0;
  while (pub_counter < 10)
  {
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub.publish (cloud);
    ros::spinOnce ();
    loop_rate.sleep ();
    pub_counter++;
  }

  // Create the target FeatureCloud with the input scene cloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::cout << std::endl << "Starting matching..." << std::endl;
  double tick1 = ros::Time::now().toSec();
  /*
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(model_cloud);
  icp.setInputTarget(msg);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  Eigen::Matrix4f Tm = icp.getFinalTransformation();
  */
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];
  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);
  Eigen::Matrix4f Tm = best_alignment.final_transformation;
  double tick2 = ros::Time::now().toSec();
  std::cout << std::endl << "Matching Elapsed Time " << (tick2 - tick1) << " sec" << std::endl;

  // Print the rotation matrix and translation vector
  tf::Vector3 origin;
  origin.setValue (static_cast<double>(Tm(0,3)), static_cast<double>(Tm(1,3)), static_cast<double>(Tm(2,3)));

  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
                static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
                static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

  double roll, pitch, yaw, rToD = 180.0 / M_PI;
  tf3d.getRPY(roll, pitch, yaw);
  std::cout << "[x, y, z, roll, pitch, yaw] = ["
            << origin.getX () << " " << origin.getY () << " " << origin.getZ () << " "
            << roll * rToD    << " " << pitch * rToD   << " " << yaw * rToD     << "] " << std::endl;
  // sleep for a while
  ros::Rate sleep_rate(10);
  sleep_rate.sleep ();
}

int main ( int argc, char** argv )
{
  ros::init(argc, argv, "ARObject");
  ros::NodeHandle node;

  // loading 3D models for objects
  object_templates.resize (0);
  loadAndTrainObjectModel ();

  // subscribe to the input point cloud topic and start to match the object templates with the input scene cloud point
  ros::Subscriber sub = node.subscribe( "/pico_flexx/points", 100, pointCloudCallback );
  //ros::AsyncSpinner spinner(4);
  //spinner.start();
  ros::spin();

  return 0;
}
