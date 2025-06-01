// PCL (Point Cloud Library) Headers
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>  // VoxelGrid filter
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>


// Third-party and Utility Headers
#include <boost/make_shared.hpp>
#include <string>
#include <sstream>
#include <vector>  
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "matplotlibcpp.h"

// ROS Headers
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>  // Odometry message
#include <sensor_msgs/PointCloud2.h>  // PointCloud2 message
#include <nav_msgs/Path.h>  // Path message
#include <geometry_msgs/PoseStamped.h>  // PoseStamped message
#include <ros/package.h>  // For ROS package management


namespace plt = matplotlibcpp;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


std::vector<double> icpFitnessScores; // To store the scores
std::vector<double> GlobalFitnessScores; // To store the scores
std::vector<float> x_translation, y_translation, z_translation;
std::vector<float> roll, pitch, yaw;
double GlobalFitnessScore;

// Declare global variables
ros::NodeHandle* nh = nullptr; // Pointer to NodeHandle
// Create publishers for odom and path
ros::Publisher odom_pub;
ros::Publisher path_pub ;
ros::Publisher pcl_pub_global_cloud ;
ros::Publisher pcl_pub_aligned_cloud ;
ros::Publisher pcl_pub_given_cloud;
ros::Publisher pcl_pub_given_cloud2;


bool mergeAllAtOnce = true; // Set this flag to true to merge all clouds at once, or false to merge in two steps
double icpFitnessScore = 0.0;
double GlobalFitnessScoreThreshold = 0.9;

// Set the global alignment method in "globalAlignmentMethod"; options are "icp", "gicp", "ndt", or "none"
// - "icp" : Iterative Closest Point alignment
// - "gicp": Generalized Iterative Closest Point alignment
// - "ndt" : Normal Distributions Transform alignment
// - "none": No alignment, directly merge the result cloud into the mergedCloud
std::string globalAlignmentMethod = "none"; 

// Path to the `given_pcd_data` folder within the ROS package
std::string package_path = ros::package::getPath("pointcloud_registration");
std::string folder_path = package_path + "/kobelco_exp_2024/5_pitch_angle_40_and_80_yaw_cycle_slow/pcd_world/";
std::string file_name = "cloud_";

// Number of clouds to load
int num_clouds = 250;


//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
	// Function to load point clouds from a folder
	void loadCloudData(const std::string &folder_path, const std::string &file_name, int num_clouds, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models) {
	    std::string extension(".pcd");

      std::cout << "loading cloud no " << std::endl;

	    // Loop through the number of clouds to load
	    for (int i = 1; i <= num_clouds; ++i) {
         
	        // Create the file name
	        std::ostringstream oss;
	        oss << folder_path << i << ".pcd";
	        std::string filepath = oss.str();

	        // Ensure the file extension is `.pcd`
	        if (filepath.size() <= extension.size())
	            continue;

	        if (filepath.compare(filepath.size() - extension.size(), extension.size(), extension) == 0) {
	            PCD m;
	            m.f_name = filepath;

	            // Attempt to load the PCD file
	            if (pcl::io::loadPCDFile(filepath, *m.cloud) == -1) {
	                PCL_ERROR("Couldn't read file %s\n", filepath.c_str());
	                continue;
	            }
              else
              {         std::cout <<  i << " , ";}

	            // Remove NaN points from the cloud
	            std::vector<int> indices;
	            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

	            // Add the model to the list
	            models.push_back(m);
	        }
	    }

            std::cout << "Data loading finished " << std::endl;
                std::cout << "Size of models vector: " << models.size() << std::endl;


	}


////////////////////////////////////////////////////////////////////////////////
/** 
 * \brief Perform point cloud registration using the Normal Distributions Transform (NDT) algorithm.
 *        NDT is effective for aligning point clouds with distinct geometric features and works by
 *        approximating the target cloud as a grid of probability distributions.
 * \param sourceCloud The source point cloud to be aligned.
 * \param targetCloud The target point cloud to align to.
 * \param alignedCloud The output aligned point cloud.
 * \param finalTransformation The resultant transformation matrix from source to target.
 * \return Fitness score of the alignment; returns -1.0 if the algorithm fails to converge.
 */
float performNDTAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceCloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &targetCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &alignedCloud,
    Eigen::Matrix4f &finalTransformation
) {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Set input clouds
    ndt.setInputSource(sourceCloud);
    ndt.setInputTarget(targetCloud);

    // Set NDT parameters
    ndt.setResolution(1.0);                  // Grid resolution in meters
    ndt.setMaximumIterations(35);           // Limit for iterations
    ndt.setStepSize(0.1);                    // Step size for optimization
    ndt.setTransformationEpsilon(1e-6);     // Convergence tolerance

    // Perform alignment
    ndt.align(*alignedCloud);

    // Check convergence and retrieve transformation
    if (!ndt.hasConverged()) {
        PCL_WARN("NDT failed to converge.\n");
        return -1.0f;
    }

    finalTransformation = ndt.getFinalTransformation();
    return ndt.getFitnessScore();
}


////////////////////////////////////////////////////////////////////////////////
/** 
 * \brief Perform point cloud registration using Generalized Iterative Closest Point (GICP).
 *        GICP is an extension of ICP that uses covariance estimation for each point to improve alignment accuracy.
 * \param sourceCloud The source point cloud to be aligned.
 * \param targetCloud The target point cloud to align to.
 * \param alignedCloud The output aligned point cloud.
 * \param finalTransformation The resultant transformation matrix from source to target.
 * \return Fitness score of the alignment; returns -1.0 if the algorithm fails to converge.
 */
float performGICPAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceCloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &targetCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &alignedCloud,
    Eigen::Matrix4f &finalTransformation
) {
    // Initialize Generalized ICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

    // Set input clouds
    gicp.setInputSource(sourceCloud);
    gicp.setInputTarget(targetCloud);

    // Define GICP parameters
    float maxCorrespondenceDistance = 0.5f;  // Adjust based on cloud scale
    int maxIterations = 50;                 // Fewer iterations for faster convergence
    double transformationEpsilon = 1e-8;    // Convergence tolerance for transformations
    double euclideanFitnessEpsilon = 1e-4;  // Stopping criteria for fitness score

    gicp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    gicp.setMaximumIterations(maxIterations);
    gicp.setTransformationEpsilon(transformationEpsilon);
    gicp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);

    // Align the source cloud to the target cloud
    gicp.align(*alignedCloud);

    // Check for convergence
    if (!gicp.hasConverged()) {
        PCL_WARN("GICP failed to converge.\n");
        return -1.0f;
    }

    // Retrieve the final transformation matrix
    finalTransformation = gicp.getFinalTransformation();

    // Return the fitness score
    return gicp.getFitnessScore();
}

////////////////////////////////////////////////////////////////////////////////
/** 
 * \brief Perform point cloud registration using the Iterative Closest Point (ICP) algorithm.
 *        ICP is a widely used algorithm for aligning point clouds by minimizing the distance between corresponding points.
 * \param sourceCloud The source point cloud to be aligned.
 * \param targetCloud The target point cloud to align to.
 * \param alignedCloud The output aligned point cloud.
 * \param finalTransformation The resultant transformation matrix from source to target.
 * \return Fitness score of the alignment; higher values indicate better alignment quality.
 */
float performICPAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceCloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &targetCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &alignedCloud,
    Eigen::Matrix4f &finalTransformation
) {
    // Initialize the Iterative Closest Point (ICP) object
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Set input clouds for ICP
    icp.setInputSource(sourceCloud);
    icp.setInputTarget(targetCloud);

    // Define ICP parameters
    float maxCorrespondenceDistance = 0.5f;
    int maxIterations = 100;
    double transformationEpsilon = 1e-8;
    double euclideanFitnessEpsilon = 1e-5;

    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    icp.setMaximumIterations(maxIterations);
    icp.setTransformationEpsilon(transformationEpsilon);
    icp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);

    // Align the source cloud to the target cloud
    icp.align(*alignedCloud);

    // Retrieve the final transformation matrix
    finalTransformation = icp.getFinalTransformation();

    // Return the fitness score
    return icp.getFitnessScore();
}

void convert_transform_to_rpy(Eigen::Matrix4f pairTransform)
{
    // extract translation and RPY:
    Eigen::Vector3f translation(pairTransform(0, 3), pairTransform(1, 3), pairTransform(2, 3));
    Eigen::Matrix3f rotation_matrix = pairTransform.block<3, 3>(0, 0);

    // Convert rotation matrix to RPY (Roll, Pitch, Yaw)
    float roll_val = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2)); // Roll
    float pitch_val = std::atan2(-rotation_matrix(2, 0), std::sqrt(rotation_matrix(2, 1) * rotation_matrix(2, 1) + rotation_matrix(2, 2) * rotation_matrix(2, 2))); // Pitch
    float yaw_val = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0)); // Yaw

    // Convert radians to degrees
    roll_val = roll_val * 180.0 / M_PI;
    pitch_val = pitch_val * 180.0 / M_PI;
    yaw_val = yaw_val * 180.0 / M_PI;

    // Push values to the respective vectors
    x_translation.push_back(translation(0));
    y_translation.push_back(translation(1));
    z_translation.push_back(translation(2));

    roll.push_back(roll_val);
    pitch.push_back(pitch_val);
    yaw.push_back(yaw_val);
}
////////////////////////////////////////////////////////////////////////////////
/** \brief Registration of a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairRegistration (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample, double &icpFitnessScore)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    // grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setLeafSize (1, 1, 1);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (10);


  // How to Adjust setKSearch?
  // Higher KSearch (e.g., 50–100): Smooths the normals and reduces noise but may lose sharp features, especially in small or detailed regions.
  // Lower KSearch (e.g., 10–30): Captures fine details but can introduce noise in normal estimation, leading to unstable results.
  // Optimization Tip: Adjust KSearch based on the density of the point cloud:

  // For sparse clouds, use a larger KSearch.
  // For dense clouds, use a smaller KSearch to capture fine geometry.


  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-10);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  reg.setMaxCorrespondenceDistance (15);
  reg.setMaximumIterations (500);


  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  for (int i = 0; i < 300; ++i)
  {
    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.05);

    prev = reg.getLastIncrementalTransformation ();
  }

    icpFitnessScore = reg.getFitnessScore();

    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
 }




int main (int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "sequential_registration");
    nh = new ros::NodeHandle();

    // Create publishers for odom and path
    odom_pub = nh->advertise<nav_msgs::Odometry>("odom", 10);
    path_pub = nh->advertise<nav_msgs::Path>("path", 10);
    pcl_pub_global_cloud = nh->advertise<sensor_msgs::PointCloud2>("global_point_cloud", 10);
    pcl_pub_aligned_cloud = nh->advertise<sensor_msgs::PointCloud2>("aligned_point_cloud", 10);
    pcl_pub_given_cloud = nh->advertise<sensor_msgs::PointCloud2>("given_point_cloud", 10);
    pcl_pub_given_cloud2 = nh->advertise<sensor_msgs::PointCloud2>("given_point_cloud2", 10);

    // Vector to store loaded point clouds
    std::vector<PCD, Eigen::aligned_allocator<PCD>> data;

    // Load the point clouds
    loadCloudData(folder_path, file_name, num_clouds, data);
    
    std::cout << "Size of data vector: " << data.size() << std::endl;

    std::cout << "Data is loaded completely ";


    // Check user input
    if (data.empty ())
    {
      PCL_ERROR ("Syntax is: <source.pcd> <target.pcd> [*]");
      PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
      return (-1);
    }
      PCL_INFO ("Loaded %d datasets.", (int)data.size ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();



    std::cout << "mergeAllAtOnce is started ...  ";



    PointCloud::Ptr result (new PointCloud), source, target, global_cloud;
    Eigen::Matrix4f pairTransform;
    // Initialize a global cloud to accumulate all transformed clouds
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";

    if (!data[0].cloud || data[0].cloud->empty()) {
    PCL_ERROR("First cloud in data is null or empty.\n");
    return -1; // Handle error appropriately
}


    global_cloud =  data[0].cloud;
    std::vector<int> nan_idx;
    pcl::removeNaNFromPointCloud(*global_cloud, *global_cloud, nan_idx);

    


  for (size_t i = 1; i < num_clouds; ++i)
  {

    source = data[i].cloud;

    // Remove NaN points from point clouds
    
    pcl::removeNaNFromPointCloud(*source, *source, nan_idx);

     std::cout << "-------------------------------" << i << std::endl;


    // Remove statistical outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK(60);
    sor.setStddevMulThresh(3);
    sor.setInputCloud(source);
    sor.filter(*source);
    // sor.setInputCloud(global_cloud);
    // sor.filter(*global_cloud);


    PointCloud::Ptr temp (new PointCloud);


    PCL_INFO("Before pairRegistration");
    PCL_INFO("Before pairRegistration: source size = %lu, global_cloud size = %lu\n", source->size(), global_cloud->size());

    pairRegistration (source, global_cloud, temp, pairTransform, true, icpFitnessScore);
    PCL_INFO("After pairRegistration: temp size = %lu\n", temp->size());


    convert_transform_to_rpy(pairTransform);


    // Check the translation (too large?)
        icpFitnessScores.push_back(icpFitnessScore);
    Eigen::Vector3f translation_norm = pairTransform.block<3, 1>(0, 3);
    std::cout << "icpFitnessScore " <<  icpFitnessScore << " translation "  << translation_norm.norm()  <<std::endl;



    // Update the global transform
    GlobalTransform *= pairTransform;


PCL_INFO("Cloud size before alignment: source = %lu, global_cloud = %lu\n", source->size(), global_cloud->size());
PCL_INFO("Pair Transform: \n");


    // Transform current pair into the global transform
    pcl::transformPointCloud(*source, *result, GlobalTransform);



    *mergedCloud += *result;
 



 
   // Downsample the merged cloud
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter2;
    voxel_filter2.setInputCloud(mergedCloud);
    voxel_filter2.setLeafSize(1.0f, 1.05f, 1.05f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter2.filter(*downsampled_merged_cloud);

    *global_cloud = *downsampled_merged_cloud; 



    sensor_msgs::PointCloud2 given_cloud_msg;
    pcl::toROSMsg(*source, given_cloud_msg);
    given_cloud_msg.header.frame_id = "map"; // Set the frame of reference for the cloud
    given_cloud_msg.header.stamp = ros::Time::now();
    pcl_pub_given_cloud.publish(given_cloud_msg);


    sensor_msgs::PointCloud2 given_cloud_msg2;
    pcl::toROSMsg(*global_cloud, given_cloud_msg2);
    given_cloud_msg2.header.frame_id = "map"; // Set the frame of reference for the cloud
    given_cloud_msg2.header.stamp = ros::Time::now();
    pcl_pub_given_cloud2.publish(given_cloud_msg2);



    // Create odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "map";

    // Extract translation
    odom_msg.pose.pose.position.x = GlobalTransform(0, 3);
    odom_msg.pose.pose.position.y = GlobalTransform(1, 3);
    odom_msg.pose.pose.position.z = GlobalTransform(2, 3);

    // Extract rotation
    Eigen::Matrix3f rotation = GlobalTransform.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();

    // Publish odometry
    odom_pub.publish(odom_msg);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    path_msg.poses.push_back(pose_stamped);
    path_pub.publish(path_msg);

    // Publish the global cloud
    sensor_msgs::PointCloud2 aligned_cloud_msg;
    
    pcl::toROSMsg(*source, aligned_cloud_msg);
    aligned_cloud_msg.header.frame_id = "map"; // Set the frame of reference for the cloud
    aligned_cloud_msg.header.stamp = ros::Time::now();
    pcl_pub_aligned_cloud.publish(aligned_cloud_msg);


    // Publish the global cloud
    sensor_msgs::PointCloud2 global_cloud_msg;
    pcl::toROSMsg(*downsampled_merged_cloud, global_cloud_msg);
    global_cloud_msg.header.frame_id = "map"; // Set the frame of reference for the cloud
    global_cloud_msg.header.stamp = ros::Time::now();
    pcl_pub_global_cloud.publish(global_cloud_msg);


    // Log the size of the merged cloud
    PCL_INFO("MergedCloud size: %d.\n", mergedCloud->points.size());

  }
  

      PCL_INFO("Loop finished: \n");

  // ros::Duration(0.02).sleep(); // Delay between messages

      PCL_INFO("Loop finished2: \n");

   // // Publish the global cloud
  sensor_msgs::PointCloud2 global_cloud_msg;
  pcl::toROSMsg(*mergedCloud, global_cloud_msg);
  global_cloud_msg.header.frame_id = "map"; // Set the frame of reference for the cloud
  global_cloud_msg.header.stamp = ros::Time::now();
  pcl_pub_global_cloud.publish(global_cloud_msg);

      PCL_INFO("Published the global cloud \n");

  std::stringstream ss;
  ss <<  folder_path << "marged.pcd";
  pcl::io::savePCDFile (ss.str (), *mergedCloud, true);
  std::cout << "meged Cloud is saved in file name: marged.pcd to "  << ss.str () << std::endl;



  double sum_icpFitnessScores = std::accumulate(icpFitnessScores.begin(), icpFitnessScores.end(), 0.0);
  double mean_icpFitnessScores = sum_icpFitnessScores / icpFitnessScores.size();
  std::cout << "Mean ICP Fitness Score: " << mean_icpFitnessScores << std::endl;


 // Construct the full file path
    std::string file_path = folder_path + "odom_data.txt";

    // Open a file to write data
    std::ofstream outfile(file_path);

    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file for writing: " << file_path << std::endl;
        return 1;
    }

    // Write data to the file (one line for each data set)
    for (size_t i = 0; i < x_translation.size(); ++i) {
        outfile << x_translation[i] << " "
                << y_translation[i] << " "
                << z_translation[i] << " "
                << roll[i] << " "
                << pitch[i] << " "
                << yaw[i] << std::endl;
    }

    // Close the file
    outfile.close();

    std::cout << "Odom Data saved to " << file_path << std::endl;


    // Create a simple line plot for translation values (XYZ)
    // Subplot 1: Translation (XYZ)
    // plt::figure();
    // plt::subplot(2, 1, 1);  // 2 rows, 1 column, plot in position 1
    // plt::named_plot("X Translation", x_translation);
    // plt::named_plot("Y Translation", y_translation);
    // plt::named_plot("Z Translation", z_translation);
    // plt::xlabel("PCD Cloud Number");
    // plt::ylabel("Translation (m)");
    // plt::title("Translation (XYZ)");
    // plt::grid(true);
    // plt::legend();

    // // // Subplot 2: RPY (Roll, Pitch, Yaw)
    // plt::subplot(2, 1, 2);  // 2 rows, 1 column, plot in position 2
    // plt::named_plot("Roll", roll);
    // plt::named_plot("Pitch", pitch);
    // plt::named_plot("Yaw", yaw);
    // plt::xlabel("PCD Cloud Number");
    // plt::ylabel("Angle (Degree)");
    // plt::title("Roll, Pitch, Yaw (RPY)");
    // plt::grid(true);
    // plt::legend();


    plt::figure();
    // First subplot for GlobalFitnessScores
    plt::subplot(2, 1, 1); // 2 rows, 1 column, first plot
    plt::plot(icpFitnessScores, "r-o");
    plt::xlabel("Iteration");
    plt::ylabel("ICP Fitness Score");
    plt::title("ICP Fitness Score Over Iterations");
    plt::grid(true);

    if (globalAlignmentMethod == "icp" || globalAlignmentMethod == "gicp" || globalAlignmentMethod == "ndt") {
    // Second subplot for icpFitnessScores
    plt::subplot(2, 1, 2); // 2 rows, 1 column, second plot
    plt::plot(GlobalFitnessScores, "g-o");
    plt::xlabel("Iteration");
    plt::ylabel("Global Fitness Score");
    plt::title("Global ICP Fitness Score Over Iterations");
    plt::grid(true);

    double sum_GlobalFitnessScores = std::accumulate(GlobalFitnessScores.begin(), GlobalFitnessScores.end(), 0.0);
    double mean_GlobalFitnessScores = sum_GlobalFitnessScores / GlobalFitnessScores.size();
    std::cout << "Mean Global ICP Fitness Score: " << mean_GlobalFitnessScores << std::endl;
  


  // Show the combined figure
  plt::show();



}
}
