#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/point_cloud.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <ros/package.h> 
#include "matplotlibcpp.h"


typedef pcl::PointXYZI PointT;

namespace plt = matplotlibcpp;



//load the pcd file and store the point cloud data to the cloud variable
void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud)
{
  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read the pcd file\n");
    std::exit(-1);
  }
  std::cout << "Loaded "
            << cloud->width << "x" << cloud->height
            << " data points from the pcd file"
            << std::endl;
}



// Function to write data to 'wall_seg_n.txt' file
void writeInlierIndicesToFile(const std::vector<int>& sIndicesSorted,
                         const std::vector<int>& iIndicesSorted,
                         const pcl::ModelCoefficients& model1, const std::stringstream& fileName) {
    // Open the file
    std::ofstream fileID(fileName.str());

    // Write the data to the file
    fileID << "Sample Indices\n";
    fileID << "Indices Size:\n";
    fileID << sIndicesSorted.size() << "\n";
    fileID << "Data:\n";
    for (size_t i = 0; i < sIndicesSorted.size(); i++) {
        fileID << sIndicesSorted[i] << "\n";
    }
    fileID << "Coefficient of the plane:\n";
    fileID << model1.values[0] << " " << model1.values[1] << " " << model1.values[2] << " " << model1.values[3] << "\n";
    fileID << "Inliers Indices\n";
    fileID << "Indices Size:\n";
    fileID << iIndicesSorted.size() << "\n";
    fileID << "Data:\n";
    for (size_t i = 0; i < iIndicesSorted.size(); i++) {
        fileID << iIndicesSorted[i] << "\n";
    }

    // Close the file
    fileID.close();
}


pcl::PointCloud<PointT>::Ptr extractLargestCluster(const pcl::PointCloud<PointT>::Ptr &input_cloud, std::vector<int> &largest_cluster_indices, float cluster_tolerance)
{
  
    // Create the KdTree object
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input_cloud);

    // Euclidean Cluster Extraction parameters
    int min_cluster_size = 100;     // Minimum number of points that a cluster should have
    int max_cluster_size = 2500000;   // Maximum number of points that a cluster should have

    // Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);


    // std::cout << "Input cloud size: " << input_cloud->size();
    // std::cout << " --- Number of cluster indices found: " << cluster_indices.size() << std::endl;
    // Find the largest cluster
    int largest_cluster_index = 0;
    size_t largest_cluster_size = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        if (cluster_indices[i].indices.size() > largest_cluster_size)
        {
            largest_cluster_size = cluster_indices[i].indices.size();
            largest_cluster_index = i;
        }
    }

    // Update the largest cluster indices in the argument
    largest_cluster_indices = cluster_indices[largest_cluster_index].indices;


    // Extract the largest cluster from the input point cloud
    pcl::PointCloud<PointT>::Ptr largest_cluster(new pcl::PointCloud<PointT>);
    for (const auto &index : largest_cluster_indices)
    {
        largest_cluster->points.push_back(input_cloud->points[index]);
    }
    largest_cluster->width = static_cast<uint32_t>(largest_cluster->points.size());
    largest_cluster->height = 1;
    largest_cluster->is_dense = true;

    return largest_cluster;
}



int main (int argc, char** argv)
{
 
  ros::init(argc, argv, "plane_extraction");
  ros::NodeHandle nh;

  std::string file_directory;  // Declare the file_directory variable


  double ground_angle_degree;
  double ground_distance_threshold;
  double min_ground_plane_size;
  double ground_plane_cluster_Tolerance;
  double wall_distance_threshold;
  double wall_angle_degree;
  double min_wall_plane_size;
  double wall_plane_cluster_Tolerance;
  int min_wall_segment_cluster_size;
  int max_wall_segment_cluster_size;
  double wall_segment_cluster_Tolerance;
  double downsampling_leaf_size;
  bool ground_plane_cluster_extraction;
  bool do_downsampling;
  std::string pcd_file_name;



  // // Retrieve parameters from the ROS parameter server
  nh.param<std::string>("pcd_file_name", pcd_file_name, "original_data.pcd");
  nh.param<double>("ground_angle_degree", ground_angle_degree, 0.0);
  nh.param<double>("ground_distance_threshold", ground_distance_threshold, 0.0);
  nh.param<double>("min_ground_plane_size", min_ground_plane_size, 0.0);
  nh.param<double>("ground_plane_cluster_Tolerance", ground_plane_cluster_Tolerance, 0.0);  // Check parameter name
  nh.param<double>("wall_distance_threshold", wall_distance_threshold, 0.0);
  nh.param<double>("wall_angle_degree", wall_angle_degree, 0.0);
  nh.param<double>("min_wall_plane_size", min_wall_plane_size, 0.0);
  nh.param<double>("wall_plane_cluster_Tolerance", wall_plane_cluster_Tolerance, 0.0);  // Check parameter name
  nh.param<int>("min_wall_segment_cluster_size", min_wall_segment_cluster_size, 0);
  nh.param<int>("max_wall_segment_cluster_size", max_wall_segment_cluster_size, 0);
  nh.param<double>("wall_segment_cluster_Tolerance", wall_segment_cluster_Tolerance, 0.0);
  nh.param<double>("downsampling_leaf_size", downsampling_leaf_size, 0);
  nh.param<bool>("ground_plane_cluster_extraction", ground_plane_cluster_extraction, false);
  nh.param<bool>("do_downsampling", do_downsampling, false);

ROS_INFO("Checking if the parameter exists: %d", nh.hasParam("file_directory"));


 // Retrieve the file_directory parameter from the ROS parameter server
  if (nh.getParam("file_directory", file_directory)) {
    ROS_INFO("file_directory: %s", file_directory.c_str());
  } else {
    ROS_ERROR("Failed to retrieve file_directory parameter from the parameter server.");
  }


  pcl::PCLPointCloud2::Ptr cloud_input (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>), cloud_input_pcl (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_g (new pcl::PointCloud<PointT>);


  // std::string pcd_file = file_directory + "original_data.pcd";
  
  std::string pcd_file = file_directory + pcd_file_name;
  std::cout << "pcd_file  = " << pcd_file << std::endl;
  load_pcd_file(pcd_file, cloud_input_pcl);



if (do_downsampling) {
 // Print the number of points before downsampling
    std::cout << "PointCloud before downsampling: " << cloud_input_pcl->width * cloud_input_pcl->height << " data points." << std::endl;

    // Create a VoxelGrid filter to downsample the point cloud
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_input_pcl);
    sor.setLeafSize(downsampling_leaf_size, downsampling_leaf_size, downsampling_leaf_size);  // Set the voxel (leaf) size. Adjust according to your needs.

    // Create a PointCloud to hold the downsampled data
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    sor.filter(*cloud_filtered);

    // Print the number of points after downsampling
    std::cout << "PointCloud after downsampling: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    cloud_input_pcl = cloud_filtered;
}



 // Populate the point cloud and save the index value in the intensity field of cloud_filtered_index
  for (size_t i = 0; i < cloud_input_pcl->size(); ++i) {
      PointT point = cloud_input_pcl->at(i); // Copy the point from cloud_filtered
      point.intensity = static_cast<float>(i); // Save the index value as intensity
      cloud_filtered->push_back(point); // Add the point to cloud_filtered_index
  }


    



    std::cout << "Finished plane and extracted_cluster_cloud "  << std::endl;
    std::cout << "==================================================================================" << std::endl;



// viewer.spin();

return (0);
}
