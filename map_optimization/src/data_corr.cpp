#include<pre_processing.hpp>
#include<key_index.hpp>
#include<graph_optimization.hpp>
#include <ros/ros.h>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <ros/package.h> // Include the ROS package header

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseArray.h"
#include <g2o/edge_plane_identity.hpp>

#include <pcl/conversions.h>

#include "matplotlibcpp.h"


// ground extraction
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <ploting_cloud_visualization_troubleshooting.hpp>

namespace plt = matplotlibcpp;

using namespace std::chrono_literals;
//using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;


visualization_msgs::MarkerArray markers;
nav_msgs::Odometry odom;

pcl::PointCloud<PointT>::Ptr plane_points_ground1(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_ground2(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_ground3(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_ground4(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_ground5(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr filterred_data (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall1 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall2 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall3 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall4 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall5 (new pcl::PointCloud<PointT>);

Eigen::VectorXf coeffs_wall1(4);
Eigen::VectorXf coeffs_wall2(4);
Eigen::VectorXf coeffs_wall3(4);
Eigen::VectorXf coeffs_wall4(4);
Eigen::VectorXf coeffs_wall5(4);
Eigen::VectorXf coeffs_ground1(4);
Eigen::VectorXf coeffs_ground2(4);
Eigen::VectorXf coeffs_ground3(4);
Eigen::VectorXf coeffs_ground4(4);
Eigen::VectorXf coeffs_ground5(4);

Eigen::VectorXf coeffs_ground = Eigen::Vector4f(0  , 0  , 1 , 0) ; 



key_index plane_idx_ground1;
key_index plane_idx_ground2;
key_index plane_idx_ground3;
key_index plane_idx_ground4;
key_index plane_idx_ground5;
key_index plane_idx_wall1;
key_index plane_idx_wall2;
key_index plane_idx_wall3;
key_index plane_idx_wall4;
key_index plane_idx_wall5;

key_index filterred_idx_ground1;
key_index filterred_idx_ground2;
key_index filterred_idx_ground3;
key_index filterred_idx_ground4;
key_index filterred_idx_ground5;
key_index filterred_idx_wall1;
key_index filterred_idx_wall2;
key_index filterred_idx_wall3;
key_index filterred_idx_wall4;
key_index filterred_idx_wall5;

key_index original_index;
key_index dis;
key_index filterred_idx;
std::vector<int> indices;

std::vector<double>distances_ground1;
std::vector<double>distances_ground2;
std::vector<double>distances_ground3;
std::vector<double>distances_ground4;
std::vector<double>distances_ground5;
std::vector<double>distances_wall1;
std::vector<double>distances_wall2;
std::vector<double>distances_wall3;
std::vector<double>distances_wall4;
std::vector<double>distances_wall5;

std::vector<double> rms_distances_ground1;
std::vector<double> rms_distances_ground2;
std::vector<double> rms_distances_ground3;
std::vector<double> rms_distances_ground4;
std::vector<double> rms_distances_ground5;
std::vector<double> rms_distances_wall1;
std::vector<double> rms_distances_wall2;
std::vector<double> rms_distances_wall3;
std::vector<double> rms_distances_wall4;
std::vector<double> rms_distances_wall5;
std::vector<double> rms_distances_all_plane;
std::vector<double> after_opti_rms_distances_all_plane;

std::vector<int> roi_index_ground1;  
std::vector<int> roi_index_ground2; 
std::vector<int> roi_index_ground3; 
std::vector<int> roi_index_ground4; 
std::vector<int> roi_index_ground5; 
std::vector<int> roi_index_wall1;
std::vector<int> roi_index_wall2;
std::vector<int> roi_index_wall3;
std::vector<int> roi_index_wall4;
std::vector<int> roi_index_wall5;

std::vector<int> inliers_idx_ground1;
std::vector<int> inliers_idx_ground2;
std::vector<int> inliers_idx_ground3;
std::vector<int> inliers_idx_ground4;
std::vector<int> inliers_idx_ground5;
std::vector<int> inlier_index_wall1;
std::vector<int> inlier_index_wall2;
std::vector<int> inlier_index_wall3;
std::vector<int> inlier_index_wall4;
std::vector<int> inlier_index_wall5;



double  floor_edge_stddev;
double floor_edge_stddev2;
double floor_edge_stddev3;
double floor_edge_stddev4;
double wall_edge_stddev1;
double wall_edge_stddev2;
double wall_edge_stddev3;
double wall_edge_stddev4;
double wall_edge_stddev5;
double plane_to_plane_edge_stddev;
double parallel_edge_stddev;

pcl::PointCloud<PointT>::Ptr merge_cloud(new pcl::PointCloud<PointT>);

double mean = 0.0;
double sd = 0.0;
int number_of_wall_planes;
int number_of_ground_planes;
int num_iteration;
 
std::string ground_info_file;


int main (int argc, char** argv)

{
  ros::init(argc, argv, "data_corra");
  ros::NodeHandle nh; 

 std::string build_folder;




  ROS_INFO("Checking if the parameter exists: %d", nh.hasParam("file_directory"));
 // Retrieve the file_directory parameter from the ROS parameter server
  if (nh.getParam("file_directory", build_folder)) {
    ROS_INFO("file_directory: %s", build_folder.c_str());
  } else {
    ROS_ERROR("Failed to retrieve build_folder parameter from the parameter server.");
  }
  // Retrieve parameters from the ROS parameter server
  nh.param<int>("number_of_wall_planes",number_of_wall_planes,1);
  nh.param<int>("number_of_ground_planes", number_of_ground_planes, 1);
  nh.param<int>("num_iteration", num_iteration, 1);
  nh.param<double>("floor_edge_stddev", floor_edge_stddev, 0);
  nh.param<double>("floor_edge_stddev2", floor_edge_stddev2, 0);
  nh.param<double>("floor_edge_stddev3", floor_edge_stddev3, 0);
  nh.param<double>("floor_edge_stddev4", floor_edge_stddev4, 0);
  nh.param<double>("wall_edge_stddev1", wall_edge_stddev1, 0);
  nh.param<double>("wall_edge_stddev2", wall_edge_stddev2, 0);
  nh.param<double>("wall_edge_stddev3", wall_edge_stddev3, 0);
  nh.param<double>("wall_edge_stddev4", wall_edge_stddev4, 0);
  nh.param<double>("wall_edge_stddev5", wall_edge_stddev5, 0);



  // Create a ROS publisher for the output point cloud
  ros::Publisher pub_original_data = nh.advertise<sensor_msgs::PointCloud2> ("original_data", 1);
  ros::Publisher pub_modified_data = nh.advertise<sensor_msgs::PointCloud2> ("modified_data", 1);
  ros::Publisher pub_smooth_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("smooth_pointcloud", 1);
  ros::Publisher markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers", 16);
  ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_array", 10);

  
  std::string pcd_file = build_folder + "original_data.pcd";
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  load_pcd_file(pcd_file, cloud);
  std::cout << "Number of cloud points recieved: " << cloud->size() << std::endl;
  std::cout << "pcl::getFieldsList(*cloud): " << pcl::getFieldsList(*cloud) << std::endl;
  original_index = gen_index_original_data(cloud); //point cloud index for the original cloud
  remove_nan_data(filterred_data, cloud, indices);  //remove the NaN data from the original cloud
  filterred_idx = gen_index_general(original_index, indices);  //re-map the index after removal of NaN data
  std::cout << "original_index.frame_id.size() " << original_index.frame_id.size() << std::endl;


    // visulise the plane and inpput point cloud
    pcl::visualization::PCLVisualizer viewer("Planar Components Point Clouds");
    viewer.setBackgroundColor(0, 0, 0); // Set background color to black
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 255, 255, 255); // Green color
    viewer.addPointCloud<PointT>(cloud, single_color, "cloud");

    std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointT>> colors;
    colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 255, 0, 0));      // Red
    colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 0, 255, 0));      // Green
    colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 0, 128, 0));       // Dark Green
    colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 255, 0, 128));     // Pink
    colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 255, 0, 255));// 2. Magenta
    colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 154, 205, 50));// 3. Yellow-Green
    colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 255, 215, 0)); // 4. Gold


    pcl::visualization::PCLVisualizer viewer_optimization("Before and after optimzation");
    viewer_optimization.setBackgroundColor(0, 0, 0); // Set background color to black
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_green(cloud, 0, 255, 0); // Green color

    // Divide the window into three viewports
    int sub_window_1, sub_window_2, sub_window_3;
    viewer_optimization.createViewPort(0.0, 0.0, 0.33, 1.0, sub_window_1); // Left viewport
    viewer_optimization.createViewPort(0.33, 0.0, 0.66, 1.0, sub_window_2); // Middle viewport
    viewer_optimization.createViewPort(0.66, 0.0, 1.0, 1.0, sub_window_3); // Right viewport

    // Add headings to each subwindow
    viewer_optimization.addText("Optimzation Before (White) and After (Green)", 10, 10, "cloud1_heading", sub_window_1);
    viewer_optimization.addText("Before Optimzation", 10, 10, "cloud2_heading", sub_window_2);
    viewer_optimization.addText("After Optimzation", 10, 10, "cloud3_heading", sub_window_3);

    // Add coordinate systems with grid to each subwindow
    viewer_optimization.addCoordinateSystem(100.0, "cloud1_cs", sub_window_1);
    viewer_optimization.addCoordinateSystem(100.0, "cloud2_cs", sub_window_2);
    viewer_optimization.addCoordinateSystem(100.0, "cloud3_cs", sub_window_3);

    viewer_optimization.addPointCloud<PointT>(cloud, single_color, "cloud_before_optimzation_win1", sub_window_1); // Attach to the left viewport
    viewer_optimization.addPointCloud<PointT>(cloud, single_color_green, "cloud_before_optimzation", sub_window_2); // Attach to the middle viewport


//----------------------------------
// The following tasks are performed in the subsequent section:
// 1. Loading and visualizing the extracted wall planes.
// 2. Reading the index file for extracted plane information.
// 3. Re-mapping the plane indices.  (map between the roi_index_wall and filterred index, and plane_index_wall)


std::string wall_info_file;
wall_info_file = build_folder + "wall_seg1.txt";
load_wall_info(wall_info_file, roi_index_wall1,inlier_index_wall1,coeffs_wall1);
pcd_file = build_folder + "wall_seg_cloud1.pcd";
load_pcd_file(pcd_file,plane_points_wall1);
viewer.addPointCloud<PointT>(plane_points_wall1, colors[0 % colors.size()], "plane_points_wall1");
plane_idx_wall1 = gen_index_general(filterred_idx, inlier_index_wall1);
// plane_idx_wall1 = gen_index_general(filterred_idx_wall1, inlier_index_wall1);


if (number_of_wall_planes >= 2) {
  wall_info_file = build_folder + "wall_seg2.txt";
  load_wall_info(wall_info_file, roi_index_wall2,inlier_index_wall2,coeffs_wall2);
  pcd_file = build_folder + "wall_seg_cloud2.pcd";
  load_pcd_file(pcd_file,plane_points_wall2);
  viewer.addPointCloud<PointT>(plane_points_wall2, colors[1 % colors.size()], "plane_points_wall2");
  plane_idx_wall2 = gen_index_general(filterred_idx, inlier_index_wall2);
  // plane_idx_wall2 = gen_index_general(filterred_idx_wall2, inlier_index_wall2);
}

if (number_of_wall_planes >= 3) {
  wall_info_file = build_folder + "wall_seg3.txt";
  load_wall_info(wall_info_file, roi_index_wall3,inlier_index_wall3,coeffs_wall3);
  pcd_file = build_folder + "wall_seg_cloud3.pcd";
  load_pcd_file(pcd_file,plane_points_wall3);
  viewer.addPointCloud<PointT>(plane_points_wall3, colors[2 % colors.size()], "plane_points_wall3");
  plane_idx_wall3 = gen_index_general(filterred_idx, inlier_index_wall3);
  // plane_idx_wall3 = gen_index_general(filterred_idx_wall3, inlier_index_wall3);
}

if (number_of_wall_planes >= 4) {
  wall_info_file = build_folder + "wall_seg4.txt";
  load_wall_info(wall_info_file, roi_index_wall4,inlier_index_wall4,coeffs_wall4);
  pcd_file = build_folder + "wall_seg_cloud4.pcd";
  load_pcd_file(pcd_file,plane_points_wall4);
  viewer.addPointCloud<PointT>(plane_points_wall4, colors[3 % colors.size()], "plane_points_wall4");
  plane_idx_wall4 = gen_index_general(filterred_idx, inlier_index_wall4);
  // plane_idx_wall4 = gen_index_general(filterred_idx_wall4, inlier_index_wall4);
}

if (number_of_wall_planes >= 5) {
  wall_info_file = build_folder + "wall_seg5.txt";
  load_wall_info(wall_info_file, roi_index_wall5,inlier_index_wall5,coeffs_wall5);
  pcd_file = build_folder +  "wall_seg_cloud5.pcd";
  load_pcd_file(pcd_file,plane_points_wall5);
  viewer.addPointCloud<PointT>(plane_points_wall5, colors[4 % colors.size()], "plane_points_wall5");
  plane_idx_wall5 = gen_index_general(filterred_idx, inlier_index_wall5);
  // plane_idx_wall5 = gen_index_general(filterred_idx_wall5, inlier_index_wall5); 
}


  // adding ground plane
  if (number_of_ground_planes >= 1) {

  std::cout << "Adding ground plane start: " << std::endl;
  ground_info_file = build_folder + "ground_seg1.txt";
  load_wall_info(ground_info_file, roi_index_ground1,inliers_idx_ground1,coeffs_ground1);
  pcd_file = build_folder + "ground_seg_cloud1.pcd";
  load_pcd_file(pcd_file,plane_points_ground1);  
  viewer.addPointCloud<PointT>(plane_points_ground1, colors[0 % colors.size()], "plane_points_ground1");
  plane_idx_ground1 = gen_index_general(filterred_idx, roi_index_ground1);     //map between the roi_index_wall and filterred index, and plane_index_wall
  // plane_idx_ground1 = gen_index_general(filterred_idx_ground1,inliers_idx_ground1);
  }

if (number_of_ground_planes >= 2) {
    ground_info_file = build_folder + "ground_seg2.txt";
    load_wall_info(ground_info_file, roi_index_ground2,inliers_idx_ground2,coeffs_ground2);
    pcd_file = build_folder + "ground_seg_cloud2.pcd";
    load_pcd_file(pcd_file,plane_points_ground2);
    viewer.addPointCloud<PointT>(plane_points_ground2, colors[1 % colors.size()], "plane_points_ground2");

    plane_idx_ground2 = gen_index_general(filterred_idx, inliers_idx_ground2);
    // plane_idx_ground2 = gen_index_general(filterred_idx_ground2,inliers_idx_ground2);
}
if (number_of_ground_planes >= 3) {
    ground_info_file = build_folder + "ground_seg3.txt";
    load_wall_info(ground_info_file, roi_index_ground3,inliers_idx_ground3,coeffs_ground3);
    pcd_file = build_folder + "ground_seg_cloud3.pcd";
    load_pcd_file(pcd_file,plane_points_ground3);
    viewer.addPointCloud<PointT>(plane_points_ground3, colors[2 % colors.size()], "plane_points_ground3");
    plane_idx_ground3 = gen_index_general(filterred_idx, inliers_idx_ground3);
    // plane_idx_ground3 = gen_index_general(filterred_idx_ground3,inliers_idx_ground3);
}
if (number_of_ground_planes >= 4) {
    ground_info_file = build_folder + "ground_seg4.txt";
    load_wall_info(ground_info_file, roi_index_ground4,inliers_idx_ground4,coeffs_ground4);
    pcd_file = build_folder + "ground_seg_cloud4.pcd";
    load_pcd_file(pcd_file,plane_points_ground4);
    viewer.addPointCloud<PointT>(plane_points_ground4, colors[3 % colors.size()], "plane_points_ground4");
    plane_idx_ground4 = gen_index_general(filterred_idx, inliers_idx_ground4);
    // plane_idx_ground4 = gen_index_general(filterred_idx_ground4,inliers_idx_ground4);
}
if (number_of_ground_planes >= 5) {
    ground_info_file = build_folder + "ground_seg5.txt";
    load_wall_info(ground_info_file, roi_index_ground5,inliers_idx_ground5,coeffs_ground5);
    pcd_file = build_folder + "ground_seg_cloud5.pcd";
    load_pcd_file(pcd_file,plane_points_ground5);
    viewer.addPointCloud<PointT>(plane_points_ground5, colors[4 % colors.size()], "plane_points_ground5");
    plane_idx_ground5 = gen_index_general(filterred_idx, inliers_idx_ground5);
    // plane_idx_ground5 = gen_index_general(filterred_idx_ground5,inliers_idx_ground5);
}
viewer.spinOnce(100, true);
viewer_optimization.spinOnce(100, true);

std::cout << " index plot 1 " << std::endl;

plt::figure();
plt::suptitle("Indices relationship betweeen planes and input plointcloud"); // add a title
plt::subplot(1, 1, 1); plt::plot(inliers_idx_ground1,  {{"label", "inliers_idx_ground1"}}); plt::legend(); plt::xlabel("number of points");  plt::ylabel("index of points"); plt::grid("true");

plot_inliers_idx_of_all_planes(inliers_idx_ground1,  inliers_idx_ground2,  inliers_idx_ground3,  inliers_idx_ground4, inliers_idx_ground5,  inlier_index_wall1,  inlier_index_wall2,  inlier_index_wall3, inlier_index_wall4, inlier_index_wall5 );



// plot_index_mapping_of_one_plane(filterred_idx, filterred_idx_wall1, plane_idx_wall1, inlier_index_wall1, roi_index_wall1);
// plot_filtered_idx_of_all_planes(filterred_idx, filterred_idx_wall1, filterred_idx_wall2, filterred_idx_wall3, filterred_idx_wall4, filterred_idx_ground1, filterred_idx_ground2, filterred_idx_ground3, filterred_idx_ground4);
plot_plane_idx_of_all_planes(filterred_idx, plane_idx_wall1, plane_idx_wall2, plane_idx_wall3, plane_idx_wall4, plane_idx_ground1, plane_idx_ground2, plane_idx_ground3, plane_idx_ground4);




generate_all_plane_from_Ccoefficients(coeffs_wall1,  plane_points_wall1, coeffs_wall2,  plane_points_wall2, coeffs_wall3,  plane_points_wall3, coeffs_wall4,  plane_points_wall4, coeffs_wall5,  plane_points_wall5, 
coeffs_ground1, plane_points_ground1,  coeffs_ground2, plane_points_ground2, coeffs_ground3, plane_points_ground3, coeffs_ground4, plane_points_ground4, coeffs_ground5, plane_points_ground5);
visualize_some_2d_scan_lines_in_3d_pointcloud(cloud, filterred_idx);
verify_visualize_plane_frame_idx(cloud, number_of_wall_planes, plane_idx_wall1, plane_idx_wall2, plane_idx_wall3, plane_idx_wall4, plane_idx_wall5, inlier_index_wall1, inlier_index_wall2, inlier_index_wall3,inlier_index_wall4,inlier_index_wall5);
verify_visualize_plane_frame_idx(cloud, number_of_ground_planes, plane_idx_ground1, plane_idx_ground2, plane_idx_ground3, plane_idx_ground4, plane_idx_ground5, inliers_idx_ground1, inliers_idx_ground2, inliers_idx_ground3, inliers_idx_ground4, inliers_idx_ground5);
verify_visualize_one_plane_frame_idx( cloud, plane_idx_ground1, inliers_idx_ground1);
plot_index_mapping_of_one_plane(filterred_idx, filterred_idx_ground1, plane_idx_ground1, inliers_idx_ground1, inliers_idx_ground1);



  std::cout << " compute_distance " << std::endl;
  compute_distance(plane_points_wall1, coeffs_wall1, distances_wall1);
  compute_distance(plane_points_wall2, coeffs_wall2, distances_wall2);
  compute_distance(plane_points_wall3, coeffs_wall3, distances_wall3);
  compute_distance(plane_points_wall4, coeffs_wall4, distances_wall4);
  compute_distance(plane_points_wall5, coeffs_wall5, distances_wall5);
  compute_distance(plane_points_ground1, coeffs_ground, distances_ground1);  //compute the distance to the plane from the fitted data
  compute_distance(plane_points_ground2, coeffs_ground, distances_ground2);  //compute the distance to the plane from the fitted data
  compute_distance(plane_points_ground3, coeffs_ground, distances_ground3);  //compute the distance to the plane from the fitted data
  compute_distance(plane_points_ground4, coeffs_ground, distances_ground4);  //compute the distance to the plane from the fitted data
  compute_distance(plane_points_ground5, coeffs_ground, distances_ground5);  //compute the distance to the plane from the fitted data

  


  std::cout << " compute_rms_distance " << std::endl;
  //compute the frame_id wise rms distance to the plane
  compute_rms_distance(rms_distances_wall1, distances_wall1,  plane_idx_wall1);
  compute_rms_distance(rms_distances_wall2, distances_wall2,  plane_idx_wall2);
  compute_rms_distance(rms_distances_wall3, distances_wall3,  plane_idx_wall3);
  compute_rms_distance(rms_distances_wall4, distances_wall4,  plane_idx_wall4);
  compute_rms_distance(rms_distances_wall5, distances_wall5,  plane_idx_wall5);
  compute_rms_distance(rms_distances_ground1, distances_ground1,  plane_idx_ground1); 
  compute_rms_distance(rms_distances_ground2, distances_ground2,  plane_idx_ground2); 
  compute_rms_distance(rms_distances_ground3, distances_ground3,  plane_idx_ground3); 
  compute_rms_distance(rms_distances_ground4, distances_ground4,  plane_idx_ground4); 
  compute_rms_distance(rms_distances_ground5, distances_ground5,  plane_idx_ground5); 


  std::cout<<" ======================================== " <<std::endl;
  std::cout << "graph_construction" << std::endl;
  std::cout<<" ======================================== " <<std::endl;


  graph_construction(coeffs_ground1);


  std::cout<<" ======================================== " <<std::endl;
  std::cout << "graph_optimization(num_iteration)" <<  std::endl;
  std::cout<<" ======================================== " <<std::endl;


  graph_optimization(num_iteration);

  if(markers_pub.getNumSubscribers()) {
      markers = create_marker_array(ros::Time::now());
      markers_pub.publish(markers);
    }

    std::cout << "display_pose()" <<  std::endl;
    display_pose(pose_array_pub);


    // plt::figure;
    //  plt::suptitle("fitness_score_vec");
    // plt::subplot(3, 1, 1); plt::plot(fitness_score_vec); plt::title("fitness_score_vec"); plt::grid("true"); 
    // plt::subplot(3, 1, 2); plt::plot(weight_wx); plt::title("weight_wx"); plt::grid("true");
    // plt::subplot(3, 1, 3); plt::plot(weight_wq);    plt::title("weight_wq"); plt::grid("true");


    // plt::figure();
    // plt::suptitle("Pose after optimization");
    // plt::subplot(3, 2, 1); plt::plot(x); plt::title("x"); plt::grid("true"); 
    // plt::subplot(3, 2, 3); plt::plot(y); plt::title("y"); plt::grid("true");
    // plt::subplot(3, 2, 5); plt::plot(z);    plt::title("z"); plt::grid("true");
    // plt::subplot(3,2,2); plt::plot(Roll);  plt::title("r");plt::grid("true");
    // plt::subplot(3,2,4); plt::plot(Pitch);  plt::title("p");plt::grid("true");
    // plt::subplot(3,2,6); plt::plot(Yaw);  plt::title("y");plt::grid("true");

    std::cout << "modified_pcd_file()" <<  std::endl;
    merge_cloud = modified_pcd_file(); 

    sensor_msgs::PointCloud2 modified_pointcloud;
    pcl::toROSMsg(*merge_cloud, modified_pointcloud);
    modified_pointcloud.header.stamp = ros::Time::now();
    modified_pointcloud.header.frame_id = "laser";



  if(pub_modified_data.getNumSubscribers()) {
      pub_modified_data.publish(modified_pointcloud);}

      

    viewer_optimization.addPointCloud<PointT>(merge_cloud, single_color_green, "cloud_after_optimzation", sub_window_3); // 
    viewer_optimization.addPointCloud<PointT>(merge_cloud, single_color_green, "cloud_after_optimzation_win1", sub_window_1); // 





    



pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_cloud;
// pcl::PointCloud<pcl::PointXYZ>::Ptr merge_cloud_xyz;

pcl::PointCloud<pcl::PointXYZ>::Ptr merge_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>); 
merge_cloud_xyz->resize(merge_cloud->size());

for (size_t i = 0; i < merge_cloud->size(); ++i) 
{ 
    merge_cloud_xyz->points[i].x=merge_cloud->points[i].x; //error 
    merge_cloud_xyz->points[i].y=merge_cloud->points[i].y; //error 
    merge_cloud_xyz->points[i].z=merge_cloud->points[i].z; //error 
}


// pcl::copyPointCloud(merge_cloud_xyz, merge_cloud);

//   smooth_cloud =  smoothing(merge_cloud_xyz);
//   std::cout<<" merge_cloud size  " << merge_cloud->size() <<std::endl;
//   std::cout<<" merge_cloud_xyz size  " << merge_cloud_xyz->size() <<std::endl;
//   std::cout<<" smooth_cloud size  "<< smooth_cloud->size() <<std::endl;


// sensor_msgs::PointCloud2 smooth_pointcloud;
//     pcl::toROSMsg(*smooth_cloud, smooth_pointcloud);
//     smooth_pointcloud.header.stamp = ros::Time::now();
//     smooth_pointcloud.header.frame_id = "laser";

//   if(pub_smooth_pointcloud.getNumSubscribers()) {
//       pub_smooth_pointcloud.publish(smooth_pointcloud);}
    


  std::cout<<" " <<std::endl;
  std::cout<<" ======================================== " <<std::endl;
  calculate_mean_sd_distances(distances_ground1, mean, sd);
  std::cout<<"Mean and SD distances for ground plane before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  calculate_mean_sd_distances(distances_wall1, mean, sd);
  std::cout<<"Mean and SD distances for wall plane-1 before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  calculate_mean_sd_distances(distances_wall2, mean, sd);
  std::cout<<"Mean and SD distances for wall plane-2 before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  calculate_mean_sd_distances(distances_wall3, mean, sd);
  std::cout<<"Mean and SD distances for wall plane-3 before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  calculate_mean_sd_distances(distances_wall4, mean, sd);
  std::cout<<"Mean and SD distances for wall plane-4 before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  calculate_mean_sd_distances(distances_wall5, mean, sd);
  std::cout<<"Mean and SD distances for wall plane-5 before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  std::cout<<" ======================================== " <<std::endl;
  std::cout<<" " <<std::endl;

  display_RSSE();

// std::transform (rms_distances_ground1.begin(), rms_distances_ground1.end(), rms_distances_ground2.begin(), rms_distances_all_plane.begin(), std::plus<double>());
// std::transform (rms_distances_wall1.begin(), rms_distances_wall1.end(),  rms_distances_all_plane.begin(), rms_distances_all_plane.begin(), std::plus<double>());

// std::transform (after_opti_rms_distances_ground1.begin(), after_opti_rms_distances_ground1.end(), after_opti_rms_distances_ground2.begin(), after_opti_rms_distances_all_plane.begin(), std::plus<double>());
// std::transform (after_opti_rms_distances_wall1.begin(), after_opti_rms_distances_wall2.end(),  after_opti_rms_distances_all_plane.begin(), after_opti_rms_distances_all_plane.begin(), std::plus<double>());
std::cout<<" ================ RSS ==================== " <<std::endl;


plt::figure();
plt::suptitle("RSS rms_distances_of_all_planes ");
plt::subplot(5,2,1); plt::plot(rms_distances_ground1,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground1,  {{"label", "After"}}); plt::title("ground1");  plt::legend();  plt::grid("true");
plt::subplot(5,2, 3); plt::plot(rms_distances_ground2,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground2,  {{"label", "After"}}); plt::title("ground2"); plt::legend(); plt::grid("true");
plt::subplot(5,2, 5); plt::plot(rms_distances_ground3,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground3,  {{"label", "After"}}); plt::title("ground3"); plt::legend(); plt::grid("true");
plt::subplot(5,2, 7); plt::plot(rms_distances_ground4,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground4,  {{"label", "After"}}); plt::title("ground4"); plt::legend(); plt::grid("true");
plt::subplot(5,2, 9); plt::plot(rms_distances_ground5,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground5,  {{"label", "After"}}); plt::title("ground5"); plt::legend(); plt::grid("true");
plt::subplot(5,2,2); plt::plot(rms_distances_wall1,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall1,  {{"label", "After"}}); plt::title("wall1"); plt::legend(); plt::grid("true");
plt::subplot(5,2,4); plt::plot(rms_distances_wall2,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall2,  {{"label", "After"}}); plt::title("wall2"); plt::legend(); plt::grid("true");
plt::subplot(5,2,6); plt::plot(rms_distances_wall3,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall3,  {{"label", "After"}}); plt::title("wall3"); plt::legend(); plt::grid("true");
plt::subplot(5,2,8); plt::plot(rms_distances_wall4,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall4,  {{"label", "After"}}); plt::title("wall4"); plt::legend(); plt::grid("true");
plt::subplot(5,2,10); plt::plot(rms_distances_wall5,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall5,  {{"label", "After"}}); plt::title("wall5"); plt::legend(); plt::grid("true");
// plt::subplot(1,1,1);plt::plot(rms_distances_all_plane,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_all_plane,  {{"label", "After"}}); plt::title("Sum"); plt::title("rms_distances_all_plane");
plt::tight_layout();


// // std::cout<<" ================= G2o error ==================== " <<std::endl;
plt::figure();
plt::suptitle("G2o error ");
plt::subplot(6, 2, 1); plt::plot(rms_g2o_error_ground1,  {{"label", "ground1"}});  plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::subplot(6, 2, 3); plt::plot(rms_g2o_error_ground2,  {{"label", "ground2"}}); plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::subplot(6, 2, 5); plt::plot(rms_g2o_error_ground3,  {{"label", "ground3"}}); plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::subplot(6, 2, 7); plt::plot(rms_g2o_error_ground4,  {{"label", "ground4"}}); plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::subplot(6, 2, 9); plt::plot(rms_g2o_error_ground5,  {{"label", "ground5"}}); plt::legend(); plt::grid("true"); plt::ylim(-1, 200);

plt::subplot(6, 2, 2); plt::plot(rms_g2o_error_wall1,  {{"label", "wall1"}});  plt::legend(); plt::grid("true");plt::ylim(-1, 200);
plt::subplot(6,2,4); plt::plot(rms_g2o_error_wall2,  {{"label", "wall2"}}); plt::legend(); plt::grid("true");plt::ylim(-1, 200);
plt::subplot(6,2,6); plt::plot(rms_g2o_error_wall3,  {{"label", "wall3"}}); plt::legend(); plt::grid("true");plt::ylim(-1, 200);
plt::subplot(6,2,8); plt::plot(rms_g2o_error_wall4,  {{"label", "wall4"}});   plt::legend(); plt::grid("true");plt::ylim(-1, 200);
plt::subplot(6,2,10); plt::plot(rms_g2o_error_wall5,  {{"label", "wall5"}});  plt::legend(); plt::grid("true"); plt::ylim(-1, 200);

plt::subplot(6,2,11); plt::plot(g2o_error_plane_identity_a,  {{"label", "a"}});  plt::legend(); 
plt::subplot(6,2,11); plt::plot(g2o_error_plane_identity_b,  {{"label", "b "}});  plt::legend(); 
plt::subplot(6,2,12); plt::plot(g2o_error_plane_identity_c,  {{"label", "c "}});  plt::legend(); 
plt::subplot(6,2,12); plt::plot(g2o_error_plane_identity_d,  {{"label", "d"}});  plt::legend(); 
plt::tight_layout();




// std::cout<<" ================= Distance ==================== " <<std::endl;
plt::figure();
plt::suptitle("Distance");
plt::subplot(6, 2, 1); plt::plot(distances_ground1,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground1,  {{"label", "After"}}); plt::title("ground");  plt::legend(); plt::grid("true");
plt::subplot(6, 2, 3); plt::plot(distances_ground2,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground2,  {{"label", "After"}}); plt::title("ground"); plt::legend(); plt::grid("true");
plt::subplot(6, 2, 5); plt::plot(distances_ground3,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground3,  {{"label", "After"}}); plt::title("ground"); plt::legend(); plt::grid("true");
plt::subplot(6, 2, 7); plt::plot(distances_ground4,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground4,  {{"label", "After"}}); plt::title("ground"); plt::legend(); plt::grid("true");
plt::subplot(6, 2, 9); plt::plot(distances_ground5,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground5,  {{"label", "After"}}); plt::title("ground"); plt::legend(); plt::grid("true");
plt::subplot(6, 2, 2); plt::plot(distances_wall1,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall1,  {{"label", "After"}}); plt::title("wall1"); plt::legend(); plt::grid("true");
plt::subplot(6,2,4); plt::plot(distances_wall2,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall2,  {{"label", "After"}}); plt::title("wall2"); plt::legend(); plt::grid("true");
plt::subplot(6,2,6); plt::plot(distances_wall3,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall3,  {{"label", "After"}}); plt::title("wall3"); plt::legend(); plt::grid("true");
plt::subplot(6,2,8); plt::plot(distances_wall4,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall4,  {{"label", "After"}}); plt::title("wall4"); plt::legend(); plt::grid("true");
plt::subplot(6,2,10); plt::plot(distances_wall5,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall5,  {{"label", "After"}}); plt::title("wall5"); plt::legend(); plt::grid("true");
plt::subplot(6,2,11); plt::plot(rms_distances_all_plane,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_all_plane,  {{"label", "After"}}); plt::title("Sum");
plt::tight_layout();

std::vector<double> node_g1(edges_ground1.size(), node_ground1);
std::vector<double> node_g2(edges_ground2.size(), node_ground2);
std::vector<double> node_g3(edges_ground3.size(), node_ground3);
std::vector<double> node_g4(edges_ground4.size(), node_ground4);
std::vector<double> node_w1(edges_wall1.size(), node_wall1);
std::vector<double> node_w2(edges_wall2.size(), node_wall2);
std::vector<double> node_w3(edges_wall3.size(), node_wall3);
std::vector<double> node_w4(edges_wall4.size(), node_wall4);
std::vector<double> node_w5(edges_wall5.size(), node_wall5);

std::cout<<" ================= Edges ==================== " <<std::endl;

plt::figure();
plt::suptitle("Edges");
plt::subplot(5, 2, 1); plt::plot(edges_ground1, node_g1,  {{"label", "ground1"}, {"marker", "o"}, {"linestyle", "--"}} );  plt::legend(); plt::grid("true"); plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5, 2, 3); plt::plot(edges_ground2, node_g2, {{"label", "ground2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true"); plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5, 2, 5); plt::plot(edges_ground3, node_g3, {{"label", "ground3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true"); plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5, 2, 7); plt::plot(edges_ground4, node_g4, {{"label", "ground3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true"); plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5, 2, 2); plt::plot(edges_wall1, node_w1, {{"label", "wall1"}, {"marker", "o"}, {"linestyle", "--"}});  plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5,2,4); plt::plot(edges_wall2,  node_w2,{{"label", "wall2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5,2,6); plt::plot(edges_wall3, node_w3,  {{"label", "wall3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5,2,8); plt::plot(edges_wall4, node_w4,  {{"label", "wall4"}, {"marker", "o"}, {"linestyle", "--"}});   plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5,2,10); plt::plot(edges_wall5,  node_w5, {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}});  plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
plt::subplot(5,2,9); plt::plot(edges_ground1, node_g1,  {{"label", "ground1"}, {"marker", "o"}, {"linestyle", "--"}}); plt::plot(edges_ground2, node_g2,  {{"label", "ground2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::grid("true");
plt::plot(edges_wall1, node_w1, {{"label", "wall1"}, {"marker", "o"}, {"linestyle", "--"}});     plt::plot(edges_wall2, node_w2,  {{"label", "wall2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::grid("true");
plt::plot(edges_wall3, node_w3,  {{"label", "wall3"}, {"marker", "o"}, {"linestyle", "--"}});      plt::plot(edges_wall4, node_w4,  {{"label", "wall4"}, {"marker", "o"}, {"linestyle", "--"}}); plt::grid("true");
plt::plot(edges_wall5, node_w5,  {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 

std::cout<<" ================= End ==================== " <<std::endl;




plt::show();
    
do 
{
        viewer.spin(); // Process events once for the PCL viewer
// viewer.viewer_frame_cloud();
std::cout << '\n' << "Press a key to continue...";
} while (std::cin.get() != '\n');


plt::close();

return 0;
}




