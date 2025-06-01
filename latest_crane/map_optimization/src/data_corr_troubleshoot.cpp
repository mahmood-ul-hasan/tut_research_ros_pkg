#include<pre_processing.hpp>
#include<key_index.hpp>
#include<graph_optimization.hpp>
#include <ros/ros.h>
#include <hdl_graph_slam/information_matrix_calculator.hpp>

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


pcl::PointCloud<PointT>::Ptr merge_cloud(new pcl::PointCloud<PointT>);

double mean = 0.0;
double sd = 0.0;


 
std::string ground_info_file;


int main (int argc, char** argv)

{


  std::cout << " coeffs_ground " << coeffs_ground(0) << " " << coeffs_ground(1) << " " << coeffs_ground(2) << " " << coeffs_ground(3) << std::endl;


  ros::init(argc, argv, "data_corra");
  ros::NodeHandle nh; 

  // Create a ROS publisher for the output point cloud
  ros::Publisher pub_original_data = nh.advertise<sensor_msgs::PointCloud2> ("original_data", 1);
  ros::Publisher pub_ground1 = nh.advertise<sensor_msgs::PointCloud2> ("ground1_plan", 1);
  ros::Publisher pub_ground2 = nh.advertise<sensor_msgs::PointCloud2> ("ground2_plan", 1);
  ros::Publisher pub_ground3 = nh.advertise<sensor_msgs::PointCloud2> ("ground3_plan", 1);
  ros::Publisher pub_ground4 = nh.advertise<sensor_msgs::PointCloud2> ("ground4_plan", 1);
  ros::Publisher pub_ground5 = nh.advertise<sensor_msgs::PointCloud2> ("ground5_plan", 1);
  ros::Publisher pub_wall1 = nh.advertise<sensor_msgs::PointCloud2> ("wall1_plan", 1);
  ros::Publisher pub_wall2 = nh.advertise<sensor_msgs::PointCloud2> ("wall2_plan", 1);
  ros::Publisher pub_wall3 = nh.advertise<sensor_msgs::PointCloud2> ("wall3_plan", 1);
  ros::Publisher pub_wall4 = nh.advertise<sensor_msgs::PointCloud2> ("wall4_plan", 1);
  ros::Publisher pub_wall5 = nh.advertise<sensor_msgs::PointCloud2> ("wall5_plan", 1);
  ros::Publisher pub_scan_line = nh.advertise<sensor_msgs::PointCloud2> ("scan_line", 1);

  ros::Publisher pub_modified_data = nh.advertise<sensor_msgs::PointCloud2> ("modified_data", 1);
  ros::Publisher pub_smooth_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("smooth_pointcloud", 1);
  ros::Publisher markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers", 16);
  ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_array", 10);

  
    // initialize PointClouds and load point cloud

  std::string pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/original_data.pcd";
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  load_pcd_file(pcd_file, cloud);
  std::cout << "Number of cloud points recieved: " << cloud->size() << std::endl;
  std::cout << "pcl::getFieldsList(*cloud): " << pcl::getFieldsList(*cloud) << std::endl;
 


  original_index = gen_index_original_data(cloud); //point cloud index for the original cloud
  remove_nan_data(filterred_data, cloud, indices);  //remove the NaN data from the original cloud
  filterred_idx = gen_index_general(original_index, indices);  //re-map the index after removal of NaN data
  

std::cout << "original_index.frame_id.size() " << original_index.frame_id.size() << std::endl;
// int dif_min, dif_max;


// for (int i = 0; i < original_index.frame_id.size(); i++ ){
//   // int i = original_index.frame_id.size() -1;
// dif_min = original_index.min_index[i]- filterred_idx.min_index[i];
// dif_max = original_index.max_index[i] - filterred_idx.max_index[i];

//   std::cout << "  " << dif_min ;
//   std::cout << "  " << dif_max ;
  // std::cout << " <original_index frame_id.size(): " <<original_index.frame_id.size() << std::endl;
  // std::cout << " last frame_id: " <<original_index.frame_id[i];
  // std::cout << " last min_index: " <<original_index.min_index[i];
  // std::cout << " last max_index: " << original_index.max_index[i] << std::endl;

  // std::cout << " <filterred_idx frame_id.size(): " <<filterred_idx.frame_id.size() << std::endl;
  // std::cout << " last frame_id: " <<filterred_idx.frame_id[i];
  // std::cout << " last min_index: " <<filterred_idx.min_index[i];
  // std::cout << " last max_index: " << filterred_idx.max_index[i] << std::endl;


// }
  // std::cout << " finished dif" << std::endl;


 

  
  //load wall infomation file
  std::string wall_info_file;
  wall_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg1.txt";
  load_wall_info(wall_info_file, roi_index_wall1,inlier_index_wall1,coeffs_wall1);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg_cloud1.pcd";
  load_pcd_file(pcd_file,plane_points_wall1);

  wall_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg3.txt";
  load_wall_info(wall_info_file, roi_index_wall2,inlier_index_wall2,coeffs_wall2);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg_cloud3.pcd";
  load_pcd_file(pcd_file,plane_points_wall2);

  wall_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg5.txt";
  load_wall_info(wall_info_file, roi_index_wall3,inlier_index_wall3,coeffs_wall3);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg_cloud5.pcd";
  load_pcd_file(pcd_file,plane_points_wall3);

  wall_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg4.txt";
  load_wall_info(wall_info_file, roi_index_wall4,inlier_index_wall4,coeffs_wall4);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg_cloud4.pcd";
  load_pcd_file(pcd_file,plane_points_wall4);

  wall_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg2.txt";
  load_wall_info(wall_info_file, roi_index_wall5,inlier_index_wall5,coeffs_wall5);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/wall_seg_cloud2.pcd";
  load_pcd_file(pcd_file,plane_points_wall5);

//---------------------------------------------------------

//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointIndicesPtr ground (new pcl::PointIndices);


// // Create the filtering object
//   pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
//   pmf.setInputCloud (cloud);
//   pmf.setMaxWindowSize (20);
//   pmf.setSlope (1.0f);
//   pmf.setInitialDistance (0.5f);
//   pmf.setMaxDistance (2.0f);
//   pmf.extract (ground->indices);

//   // Create the filtering object
//   pcl::ExtractIndices<pcl::PointXYZI> extract;
//   extract.setInputCloud (cloud);
//   extract.setIndices (ground);
//   extract.filter (*cloud_filtered);

//   std::cerr << "Ground cloud after filtering: " << std::endl;
//   std::cerr << *cloud_filtered << std::endl;

//   // pcl::PCDWriter writer;
//   // writer.write<pcl::PointXYZI> ("ground_seg_cloud0.pcd", *cloud_filtered, false);
//   pcl::io::savePCDFileASCII("/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg_cloud00.pcd", *cloud_filtered);
//   std::cerr << "Saved " << cloud_filtered->points.size() << " data points to ground_seg_cloud0.pcd." << std::endl;






  //----------------------------------
  // adding ground plane
  std::cout << "Adding ground plane start: " << std::endl;
  ground_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg1.txt";
  load_wall_info(ground_info_file, roi_index_ground1,inliers_idx_ground1,coeffs_ground1);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg_cloud1.pcd";
  load_pcd_file(pcd_file,plane_points_ground1);
  // plane_points_ground1 = cloud_filtered;

  ground_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg4_1.txt";
  load_wall_info(ground_info_file, roi_index_ground2,inliers_idx_ground2,coeffs_ground2);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg_cloud4_1.pcd";
  load_pcd_file(pcd_file,plane_points_ground2);

  ground_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg3__.txt";
  load_wall_info(ground_info_file, roi_index_ground3,inliers_idx_ground3,coeffs_ground3);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg_cloud3__.pcd";
  load_pcd_file(pcd_file,plane_points_ground3);

  ground_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg2.txt";
  load_wall_info(ground_info_file, roi_index_ground4,inliers_idx_ground4,coeffs_ground4);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg_cloud2.pcd";
  load_pcd_file(pcd_file,plane_points_ground4);

  ground_info_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg5.txt";
  load_wall_info(ground_info_file, roi_index_ground5,inliers_idx_ground5,coeffs_ground5);
  pcd_file = "/home/aisl/catkin_ws/src/latest_crane/map_optimization/build/ground_seg_cloud5.pcd";
  load_pcd_file(pcd_file,plane_points_ground5);


  // std::cout << "coeffs_wall1: " << coeffs_wall1 << std::endl;
  // std::cout << "roi_index_wall1: " << roi_index_wall1 << std::endl;
  // std::cout << "inlier_index_wall1: " << inlier_index_wall1 << std::endl;
  // std::cout << "coeffs_wall3: " << coeffs_wall3 << std::endl;
  // std::cout << "coeffs_wall4: " << coeffs_wall4 << std::endl;

  //load fitted wall points from wall_seg_cloud.pcd file
 





 
  //voxel_grid_filter(filterred_cloud, filterred_data);
  
  //ground plane extraction
  //fit_plane(filterred_data,plane_points,coeffs,inliers_idx);  //fit the plane to the cloud data 
  
std::vector<double> orig(edges_ground2.size(), node_ground2);



// plt::figure();
// plt::suptitle("roi index for each plane"); // add a title
// plt::subplot(4, 2, 1); plt::plot(roi_index_ground1,  {{"label", "ground1"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
// plt::subplot(4, 2, 2); plt::plot(roi_index_ground2,  {{"label", "ground2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points");plt::grid("true");
// plt::subplot(4, 2, 3); plt::plot(roi_index_wall1,  {{"label", "wall1"}, {"marker", "o"}, {"linestyle", "--"}});plt::legend();  plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");
// plt::subplot(4, 2, 4); plt::plot(roi_index_wall2,  {{"label", "wall2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points");plt::grid("true");
// plt::subplot(4, 2, 5); plt::plot(roi_index_wall3,  {{"label", "wall3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points");plt::grid("true");
// plt::subplot(4, 2, 6); plt::plot(roi_index_wall4,  {{"label", "wall4"}, {"marker", "o"}, {"linestyle", "--"}});plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");
// plt::subplot(4, 2, 7); plt::plot(roi_index_wall5,  {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");
// plt::subplot(4, 2, 8); plt::plot(filterred_idx.max_index,  {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points");plt::grid("true");
// plt::subplot(4, 2, 8); plt::plot(roi_index_ground1,  {{"label", "plane_idx_ground1"}});
// plt::plot(roi_index_ground2,  {{"label", "plane_idx_ground2"}});
// plt::plot(roi_index_wall1,  {{"label", "plane_idx_wall1"}});
// plt::plot(roi_index_wall2,  {{"label", "plane_idx_wall2"}});
// plt::plot(roi_index_wall3,  {{"label", "plane_idx_wall3"}});
// plt::plot(roi_index_wall4,  {{"label", "plane_idx_wall4"}});
// plt::plot(roi_index_wall5,  {{"label", "plane_idx_wall5"}});
// plt::legend();  
// plt::tight_layout();





// plt::figure();
// plt::suptitle("inliers_idx"); // add a title
// plt::subplot(4, 2, 1); plt::plot(inliers_idx_ground1,  {{"label", "ground1"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 2); plt::plot(inliers_idx_ground2,  {{"label", "ground2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 3); plt::plot(inlier_index_wall1,  {{"label", "wall1"}, {"marker", "o"}, {"linestyle", "--"}});plt::legend();  
// plt::subplot(4, 2, 4); plt::plot(inlier_index_wall2,  {{"label", "wall2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 5); plt::plot(inlier_index_wall3,  {{"label", "wall3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 6); plt::plot(inlier_index_wall4,  {{"label", "wall4"}, {"marker", "o"}, {"linestyle", "--"}});plt::legend(); 
// plt::subplot(4, 2, 7); plt::plot(inlier_index_wall5,  {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 8); plt::plot(inliers_idx_ground1,  {{"label", "plane_idx_ground1"}});
// plt::plot(inliers_idx_ground2,  {{"label", "plane_idx_ground2"}});
// plt::plot(inlier_index_wall1,  {{"label", "plane_idx_wall1"}});
// plt::plot(inlier_index_wall2,  {{"label", "plane_idx_wall2"}});
// plt::plot(inlier_index_wall3,  {{"label", "plane_idx_wall3"}});
// plt::plot(inlier_index_wall4,  {{"label", "plane_idx_wall4"}});
// plt::plot(inlier_index_wall5,  {{"label", "plane_idx_wall5"}}); plt::legend(); 




//==================================================
  //publish different plan as point cloud
  
  // publish the  orignal cloud
  
  std::cout << " publishing  point cloud " << std::endl;

  sensor_msgs::PointCloud2 original_data_pointcloud;
  pcl::toROSMsg(*cloud, original_data_pointcloud);
  original_data_pointcloud.header.stamp = ros::Time::now();
  original_data_pointcloud.header.frame_id = "laser";

  if(pub_original_data.getNumSubscribers()) {
      pub_original_data.publish(original_data_pointcloud);}

  sensor_msgs::PointCloud2 ground1_plan_pointcloud;
  pcl::toROSMsg(*plane_points_ground1, ground1_plan_pointcloud);
  ground1_plan_pointcloud.header.stamp = ros::Time::now();
  ground1_plan_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 ground2_plan_pointcloud;
  pcl::toROSMsg(*plane_points_ground2, ground2_plan_pointcloud);
  ground2_plan_pointcloud.header.stamp = ros::Time::now();
  ground2_plan_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 ground3_plan_pointcloud;
  pcl::toROSMsg(*plane_points_ground3, ground3_plan_pointcloud);
  ground3_plan_pointcloud.header.stamp = ros::Time::now();
  ground3_plan_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 ground4_plan_pointcloud;
  pcl::toROSMsg(*plane_points_ground4, ground4_plan_pointcloud);
  ground4_plan_pointcloud.header.stamp = ros::Time::now();
  ground4_plan_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 ground5_plan_pointcloud;
  pcl::toROSMsg(*plane_points_ground5, ground5_plan_pointcloud);
  ground5_plan_pointcloud.header.stamp = ros::Time::now();
  ground5_plan_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 pub_wall1_pointcloud;
  pcl::toROSMsg(*plane_points_wall1, pub_wall1_pointcloud);
  pub_wall1_pointcloud.header.stamp = ros::Time::now();
  pub_wall1_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 pub_wall2_pointcloud;
  pcl::toROSMsg(*plane_points_wall2, pub_wall2_pointcloud);
  pub_wall2_pointcloud.header.stamp = ros::Time::now();
  pub_wall2_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 pub_wall3_pointcloud;
  pcl::toROSMsg(*plane_points_wall3, pub_wall3_pointcloud);
  pub_wall3_pointcloud.header.stamp = ros::Time::now();
  pub_wall3_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 pub_wall4_pointcloud;
  pcl::toROSMsg(*plane_points_wall4, pub_wall4_pointcloud);
  pub_wall4_pointcloud.header.stamp = ros::Time::now();
  pub_wall4_pointcloud.header.frame_id = "laser";

  sensor_msgs::PointCloud2 pub_wall5_pointcloud;
  pcl::toROSMsg(*plane_points_wall5, pub_wall5_pointcloud);
  pub_wall5_pointcloud.header.stamp = ros::Time::now();
  pub_wall5_pointcloud.header.frame_id = "laser";


  for (unsigned int i = 0; i <= 15 ; i++) {
                  pub_ground1.publish(ground1_plan_pointcloud);
                  pub_ground2.publish(ground2_plan_pointcloud);
                  pub_ground3.publish(ground3_plan_pointcloud);
                  pub_ground4.publish(ground4_plan_pointcloud);
                  pub_ground5.publish(ground5_plan_pointcloud);
                  pub_wall1.publish(pub_wall1_pointcloud);
                  pub_wall2.publish(pub_wall2_pointcloud);
                  pub_wall3.publish(pub_wall3_pointcloud);
                  pub_wall4.publish(pub_wall4_pointcloud);
                  pub_wall5.publish(pub_wall5_pointcloud);
                   }
  //------------------------------------------------------
  
  


  // ======================
  // save the ground plane
  // std::cout << "Saving GP PCD: " << std::endl;
  // pcl::io::savePCDFileASCII ("ground_plane_pcd.pcd", *plane_points);
  // std::cerr << "Saved " << plane_points->points.size () << " data points to ground_plane_pcd.pcd." << std::endl;

  
  std::cout << " starting index " << std::endl;

  //map between the roi_index_wall and filterred index, and plane_index_wall
  filterred_idx_ground1 = gen_index_general(filterred_idx, roi_index_ground1);
  plane_idx_ground1 = gen_index_general(filterred_idx_ground1,inliers_idx_ground1);
  
  filterred_idx_ground2 = gen_index_general(filterred_idx, roi_index_ground2);
  plane_idx_ground2 = gen_index_general(filterred_idx_ground2,inliers_idx_ground2);
 
  filterred_idx_ground3 = gen_index_general(filterred_idx, roi_index_ground3);
  plane_idx_ground3 = gen_index_general(filterred_idx_ground3,inliers_idx_ground3);

   filterred_idx_ground4 = gen_index_general(filterred_idx, roi_index_ground4);
  plane_idx_ground4 = gen_index_general(filterred_idx_ground4,inliers_idx_ground4);

   filterred_idx_ground5 = gen_index_general(filterred_idx, roi_index_ground5);
  plane_idx_ground5 = gen_index_general(filterred_idx_ground5,inliers_idx_ground5);

//  filter //map between the roi_index_wall and filterred index, and plane_index_wall
  filterred_idx_wall1 = gen_index_general(filterred_idx, roi_index_wall1);
  plane_idx_wall1 = gen_index_general(filterred_idx_wall1, inlier_index_wall1);
  filterred_idx_wall2 = gen_index_general(filterred_idx, roi_index_wall2);
  plane_idx_wall2 = gen_index_general(filterred_idx_wall2, inlier_index_wall2);
  filterred_idx_wall3 = gen_index_general(filterred_idx, roi_index_wall3);
  plane_idx_wall3 = gen_index_general(filterred_idx_wall3, inlier_index_wall3);
  filterred_idx_wall4 = gen_index_general(filterred_idx, roi_index_wall4);
  plane_idx_wall4 = gen_index_general(filterred_idx_wall4, inlier_index_wall4);
  filterred_idx_wall5 = gen_index_general(filterred_idx, roi_index_wall5);
  plane_idx_wall5 = gen_index_general(filterred_idx_wall5, inlier_index_wall5); 

  std::cout << " figure " << std::endl;
  
//   pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
//   pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
//   std::vector<double> frame_pts_distances;
//   std::vector<double> ground_plane2_dis;
//   std::vector<double> ground_plane2_points;
// for (int vertexID = 0; vertexID<= plane_idx_ground2.frame_id.size(); vertexID++){
//   double dist_g = 0, rms_dist_g =0;

// if((plane_idx_ground2.min_index[vertexID] != -1) && (plane_idx_ground2.max_index[vertexID] != -1)){

//   extract_frame_cloud(frame_cloud, plane_points_ground2, plane_idx_ground2, vertexID);

//   compute_distance(frame_cloud, coeffs_ground2, frame_pts_distances);
//   //compute the rms distance
//   for(size_t i = 0; i<(plane_idx_ground2.max_index[vertexID-1] - plane_idx_ground2.min_index[vertexID-1] + 1); i++)
//     {dist_g+= pow(frame_pts_distances[i],2);
//     ground_plane2_points.push_back(frame_pts_distances[i]);
//     }
//   rms_dist_g = sqrt(dist_g);
//   ground_plane2_dis.push_back(rms_dist_g);
//   std::cout<<"F_ID:"<<vertexID<<" size frame: "<< frame_cloud->points.size() <<" size points: "<< frame_pts_distances.size()<< " max_i "<<(plane_idx_ground2.max_index[vertexID-1] - plane_idx_ground2.min_index[vertexID-1] + 1) << " rms_dist: "<< rms_dist_g <<" dist " <<dist_g << "\n";

//   // std::cout<<"F_ID:"<<vertexID<<" rms_dist diff ground plane2: "<< rms_dist <<"  " <<dist << "\n";
// }  
// else
// {
//   dist_g = 0;
//    ground_plane2_dis.push_back(sqrt(dist_g));
//        ground_plane2_points.push_back(0);

// continue;

// }
 
// }
// plt::figure();
// plt::subplot(2, 1, 1); plt::plot(ground_plane2_dis);
// plt::subplot(2, 1, 2); plt::plot(ground_plane2_points);



// plt::figure();
// plt::suptitle("filterred_idx"); // add a title
// plt::subplot(4, 2, 1); plt::plot(filterred_idx_ground1.frame_id, filterred_idx_ground1.max_index,  {{"label", "ground1"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 2); plt::plot(filterred_idx_ground2.frame_id, filterred_idx_ground2.max_index,  {{"label", "ground2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 3); plt::plot(filterred_idx_wall1.frame_id, filterred_idx_wall1.max_index,  {{"label", "wall1"}, {"marker", "o"}, {"linestyle", "--"}});plt::legend();  
// plt::subplot(4, 2, 4); plt::plot(filterred_idx_wall2.frame_id, filterred_idx_wall2.max_index,  {{"label", "wall2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 5); plt::plot(filterred_idx_wall3.frame_id, filterred_idx_wall3.max_index,  {{"label", "wall3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 
// plt::subplot(4, 2, 6); plt::plot(filterred_idx_wall4.frame_id, filterred_idx_wall4.max_index,  {{"label", "wall4"}, {"marker", "o"}, {"linestyle", "--"}});plt::legend(); 
// plt::subplot(4, 2, 7); plt::plot(filterred_idx_wall5.frame_id, filterred_idx_wall5.max_index,  {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 




// plt::figure();
// plt::suptitle("plane index after mapping for tracking each scan line"); // add a title
// plt::subplot(4, 2, 1); plt::plot(plane_idx_ground1.frame_id, plane_idx_ground1.max_index,  {{"label", "ground1"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true"); plt::xlabel("number of scanlines"); plt::ylabel("index of points");
// plt::subplot(4, 2, 2); plt::plot(plane_idx_ground2.frame_id, plane_idx_ground2.max_index,  {{"label", "ground2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true"); plt::xlabel("number of scanlines"); plt::ylabel("index of points");
// plt::subplot(4, 2, 3); plt::plot(plane_idx_ground3.frame_id, plane_idx_ground3.max_index,  {{"label", "wall1"}, {"marker", "o"}, {"linestyle", "--"}});plt::legend();  plt::grid("true"); plt::xlabel("number of scanlines"); plt::ylabel("index of points");
// plt::subplot(4, 2, 4); plt::plot(plane_idx_ground4.frame_id, plane_idx_ground4.max_index,  {{"label", "wall2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true");plt::xlabel("number of scanlines"); plt::ylabel("index of points");
// plt::subplot(4, 2, 5); plt::plot(plane_idx_ground5.frame_id, plane_idx_ground5.max_index,  {{"label", "wall3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true");plt::xlabel("number of scanlines"); plt::ylabel("index of points");
// plt::subplot(4, 2, 6); plt::plot(plane_idx_wall4.frame_id, plane_idx_wall4.max_index,  {{"label", "wall4"}, {"marker", "o"}, {"linestyle", "--"}});plt::legend(); plt::grid("true");
// plt::subplot(4, 2, 7); plt::plot(plane_idx_wall5.frame_id, plane_idx_wall5.max_index,  {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true");plt::xlabel("number of scanlines"); plt::ylabel("index of points");
// plt::subplot(4, 2, 8); plt::plot(filterred_idx.frame_id, filterred_idx.max_index,  {{"label", "orignal PCD"}}); plt::legend(); plt::legend(); plt::grid("true");plt::xlabel("number of scanlines"); plt::ylabel("index of points");
// plt::tight_layout();

// plt::plot(plane_idx_ground1.frame_id, plane_idx_ground1.max_index,  {{"label", "plane_idx_ground1"}});
// plt::plot(plane_idx_ground2.frame_id, plane_idx_ground2.max_index,  {{"label", "plane_idx_ground2"}});
// plt::plot(plane_idx_wall1.frame_id, plane_idx_wall1.max_index,  {{"label", "plane_idx_wall1"}});
// plt::plot(plane_idx_wall2.frame_id, plane_idx_wall2.max_index,  {{"label", "plane_idx_wall2"}});
// plt::plot(plane_idx_wall3.frame_id, plane_idx_wall3.max_index,  {{"label", "plane_idx_wall3"}});
// plt::plot(plane_idx_wall4.frame_id, plane_idx_wall4.max_index,  {{"label", "plane_idx_wall4"}});
// plt::plot(plane_idx_wall4.frame_id, plane_idx_wall4.min_index,  {{"label", "min plane_idx_wall5"}}); plt::legend(); 


  //compute distance to wall from fitted points and also compute the frame_wise rms distances
 
// for(int i= 0; i< roi_index_wall1.size(); i++)
//   {
//       std::cout << "  " <<roi_index_wall1[i];
//   }


std::cout << " <plane_points_ground1 size(): " <<plane_points_ground1->size() << std::endl;
  std::cout << " <inliers_idx_ground1 size(): " <<inliers_idx_ground1.size() << std::endl;
  std::cout << " <roi_index_ground1 size(): " <<roi_index_ground1.size() << std::endl;
  std::cout << " <plane_idx_ground1 frame_id.size(): " <<plane_idx_ground1.frame_id.size() << std::endl;
  std::cout << " last element plane_idx_ground1 : " <<plane_idx_ground1.frame_id[plane_idx_ground1.frame_id.size()-1] << std::endl;


  std::cout << " finish index " << std::endl;

  std::vector<int> point_indices;
  pcl::PointCloud<PointT>::Ptr extracted_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr extracted_cloud1 (new pcl::PointCloud<PointT>);
  extract_cloud(extracted_cloud, 0);
  extract_cloud(extracted_cloud1, 1);
std::vector<double> extracted_cloud_points;
std::vector<double> extracted_cloud1_points;
 for (size_t i = 0; i < extracted_cloud->points.size(); ++i)
  { extracted_cloud_points.push_back(extracted_cloud->points[i].x);
  extracted_cloud1_points.push_back(extracted_cloud1->points[i].x);
  }

  

  int k = 0;
  key_index current_index;
  current_index = plane_idx_ground1;

for(int i= 0; i< current_index.frame_id.size(); i++)
  {
      // std::cout << " filterred_wall frame_id.size(): " <<current_index.frame_id.size();
      // std::cout << " frame_id: " <<current_index.frame_id[i];
      // std::cout << " min_index: " <<current_index.min_index[i];
      // std::cout << " max_index: " << current_index.max_index[i] << std::endl;

      // std::cout << " <filterred_org frame_id.size(): " <<filterred_idx.frame_id.size();
      // std::cout << " frame_id: " <<filterred_idx.frame_id[i];
      // std::cout << " min_index: " <<filterred_idx.min_index[i];
      // std::cout << " max_index: " << filterred_idx.max_index[i] << std::endl;

      // std::fill(filterred_idx.min_index[1],filterred_idx.max_index[1],0);

      //  std::cout << " point_indices : " <<  std::endl;;

       for(int j = current_index.min_index[i]; j < current_index.max_index[i]; j++)
  {
    point_indices.push_back(j);
    //  std::cout << " " << point_indices[k];
    //       k = k+1;
  }

      pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      sensor_msgs::PointCloud2 scan_line_pointcloud;

      // pcl::copyPointCloud(*cloud, point_indices, *extracted_cloud);
        extract_cloud(extracted_cloud, i);
      // std::cout << " i: " << i;

      pcl::toROSMsg(*extracted_cloud, scan_line_pointcloud);
      scan_line_pointcloud.header.stamp = ros::Time::now();
      scan_line_pointcloud.header.frame_id = "laser";
      if(pub_scan_line.getNumSubscribers()) {
      pub_scan_line.publish(scan_line_pointcloud);}

      // ros::Duration(0.1).sleep();
      point_indices.clear();
      k = 0;
  }

// point_indices = roi_index_ground1;
      // pcl::copyPointCloud(*cloud, point_indices, *extracted_cloud);
      // sensor_msgs::PointCloud2 scan_line_pointcloud;
      // pcl::toROSMsg(*extracted_cloud, scan_line_pointcloud);
      // scan_line_pointcloud.header.stamp = ros::Time::now();
      // scan_line_pointcloud.header.frame_id = "laser";
      // if(pub_scan_line.getNumSubscribers()) {
      // pub_scan_line.publish(scan_line_pointcloud);}
  
 

  

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

  /*for(int i:original_index.frame_id){
    std::cout<<filterred_idx.frame_id[i]<<"  "<<filterred_idx.min_index[i]<<"  "<<filterred_idx.max_index[i]<<"     ";
    std::cout<<filterred_idx_wall5.frame_id[i]<<"  "<<filterred_idx_wall5.min_index[i]<<"  "<<filterred_idx_wall5.max_index[i]<<"   ";
    std::cout<<plane_idx_wall5.frame_id[i]<<"  "<<plane_idx_wall5.min_index[i]<<"  "<<plane_idx_wall5.max_index[i]<<std::endl;
    //std::cout<<rms_distances[i]<<std::endl;
  }*/
  


  //display some distance data  
  //for(auto i:distances_wall5)
    //std::cout<<i<<std::endl;
  //std::cout<<"Data size: "<<distances.size();

   
  std::cout<<" ======================================== " <<std::endl;
  std::cout << "graph_construction" << std::endl;
  std::cout<<" ======================================== " <<std::endl;


  graph_construction(coeffs_ground1);

  //   if(markers_pub.getNumSubscribers()) {
  //     markers = create_marker_array(ros::Time::now());
  //     markers_pub.publish(markers);
  //   }
  //     display_pose(pose_array_pub);

  std::cout<<" ======================================== " <<std::endl;
    std::cout << "graph_optimization()" <<  std::endl;
      std::cout<<" ======================================== " <<std::endl;


  graph_optimization();

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


    plt::figure();
    plt::suptitle("Pose after optimization");
    plt::subplot(3, 2, 1); plt::plot(x); plt::title("x"); plt::grid("true"); 
    plt::subplot(3, 2, 3); plt::plot(y); plt::title("y"); plt::grid("true");
    plt::subplot(3, 2, 5); plt::plot(z);    plt::title("z"); plt::grid("true");
    plt::subplot(3,2,2); plt::plot(Roll);  plt::title("r");plt::grid("true");
    plt::subplot(3,2,4); plt::plot(Pitch);  plt::title("p");plt::grid("true");
    plt::subplot(3,2,6); plt::plot(Yaw);  plt::title("y");plt::grid("true");

    std::cout << "modified_pcd_file()" <<  std::endl;
    merge_cloud = modified_pcd_file(); 

    sensor_msgs::PointCloud2 modified_pointcloud;
    pcl::toROSMsg(*merge_cloud, modified_pointcloud);
    modified_pointcloud.header.stamp = ros::Time::now();
    modified_pointcloud.header.frame_id = "laser";

  if(pub_modified_data.getNumSubscribers()) {
      pub_modified_data.publish(modified_pointcloud);}

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
plt::suptitle("RSS");
plt::subplot(4, 2, 1); plt::plot(rms_distances_ground1,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground1,  {{"label", "After"}}); plt::title("ground");  plt::legend();  plt::grid("true");
plt::subplot(4, 2, 2); plt::plot(rms_distances_ground2,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground2,  {{"label", "After"}}); plt::title("ground"); plt::legend(); plt::grid("true");
plt::subplot(4, 2, 3); plt::plot(rms_distances_wall1,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall1,  {{"label", "After"}}); plt::title("wall1"); plt::legend(); plt::grid("true");
plt::subplot(4,2,4); plt::plot(rms_distances_wall2,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall2,  {{"label", "After"}}); plt::title("wall2"); plt::legend(); plt::grid("true");
plt::subplot(4,2,5); plt::plot(rms_distances_wall3,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall3,  {{"label", "After"}}); plt::title("wall3"); plt::legend(); plt::grid("true");
plt::subplot(4,2,6); plt::plot(rms_distances_wall4,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall4,  {{"label", "After"}}); plt::title("wall4"); plt::legend(); plt::grid("true");
plt::subplot(4,2,7); plt::plot(rms_distances_wall5,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_wall5,  {{"label", "After"}}); plt::title("wall5"); plt::legend(); plt::grid("true");
// plt::subplot(4,2,8); plt::plot(rms_distances_all_plane,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_all_plane,  {{"label", "After"}}); plt::title("Sum");
plt::tight_layout();
// std::cout<<" ================= G2o error ==================== " <<std::endl;

// plt::figure();
// plt::suptitle("RSS ground");
// plt::subplot(4, 2, 1); plt::plot(rms_distances_ground1,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground1,  {{"label", "After"}}); plt::title("ground1");  plt::legend();  plt::grid("true");
// plt::subplot(4, 2, 2); plt::plot(rms_distances_ground2,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground2,  {{"label", "After"}}); plt::title("ground2"); plt::legend(); plt::grid("true");
// plt::subplot(4, 2, 3); plt::plot(rms_distances_ground3,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground3,  {{"label", "After"}}); plt::title("ground3"); plt::legend(); plt::grid("true");
// plt::subplot(4,2,4); plt::plot(rms_distances_ground4,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground4,  {{"label", "After"}}); plt::title("ground4"); plt::legend(); plt::grid("true");
// plt::subplot(4,2,5); plt::plot(rms_distances_ground5,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_ground5,  {{"label", "After"}}); plt::title("ground5"); plt::legend(); plt::grid("true");
// plt::tight_layout();



// std::cout<<" ================= G2o error ==================== " <<std::endl;
plt::figure();
plt::suptitle("G2o error ");
plt::subplot(4, 2, 1); plt::plot(rms_g2o_error_ground1,  {{"label", "ground1"}});  plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::subplot(4, 2, 2); plt::plot(rms_g2o_error_ground2,  {{"label", "ground2"}}); plt::legend(); plt::grid("true"); plt::ylim(-1, 50);
plt::subplot(4, 2, 3); plt::plot(rms_g2o_error_wall1,  {{"label", "wall1"}});  plt::legend(); plt::grid("true");plt::ylim(-1, 50);
plt::subplot(4,2,4); plt::plot(rms_g2o_error_wall2,  {{"label", "wall2"}}); plt::legend(); plt::grid("true");plt::ylim(-1, 50);
plt::subplot(4,2,5); plt::plot(rms_g2o_error_wall3,  {{"label", "wall3"}}); plt::legend(); plt::grid("true");plt::ylim(-1, 50);
plt::subplot(4,2,6); plt::plot(rms_g2o_error_wall4,  {{"label", "wall4"}});   plt::legend(); plt::grid("true");plt::ylim(-1, 50);
plt::subplot(4,2,7); plt::plot(rms_g2o_error_wall5,  {{"label", "wall5"}});  plt::legend(); plt::grid("true"); plt::ylim(-1, 50);
plt::subplot(4,2,8); plt::plot(g2o_error_plane_identity_a,  {{"label", "a"}});  plt::legend(); 
plt::subplot(4,2,8); plt::plot(g2o_error_plane_identity_b,  {{"label", "b "}});  plt::legend(); 
plt::subplot(4,2,8); plt::plot(g2o_error_plane_identity_c,  {{"label", "c "}});  plt::legend(); 
plt::subplot(4,2,8); plt::plot(g2o_error_plane_identity_d,  {{"label", "d"}});  plt::legend(); 
plt::tight_layout();


plt::figure();
plt::suptitle("G2o error Ground");
plt::subplot(4, 2, 1); plt::plot(rms_g2o_error_ground,  {{"label", "ground_all"}});  plt::legend(); plt::grid("true");  plt::ylim(-1, 200);
plt::subplot(4, 2, 2); plt::plot(rms_g2o_error_ground1,  {{"label", "ground1"}}); plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::subplot(4, 2, 3); plt::plot(rms_g2o_error_ground2,  {{"label", "ground2"}});  plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::subplot(4,2,4); plt::plot(rms_g2o_error_ground3,  {{"label", "ground3"}}); plt::legend(); plt::grid("true");   plt::ylim(-1, 200);
plt::subplot(4,2,5); plt::plot(rms_g2o_error_ground4,  {{"label", "ground4"}}); plt::legend(); plt::grid("true");  plt::ylim(-1, 200);
plt::subplot(4,2,6); plt::plot(rms_g2o_error_ground5,  {{"label", "ground5"}}); plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::tight_layout();


plt::figure();
plt::plot(rms_g2o_error_ground,  {{"label", "ground all"}}); plt::legend(); plt::grid("true"); plt::ylim(-1, 200);
plt::plot(rms_g2o_error_ground1,  {{"label", "ground1"}}); plt::legend(); plt::grid("true"); //plt::ylim(-1, 2000);
plt::plot(rms_g2o_error_ground2,  {{"label", "ground2"}}); plt::legend(); plt::grid("true"); //plt::ylim(-1, 2000);
plt::plot(rms_g2o_error_ground3,  {{"label", "ground3"}}); plt::legend(); plt::grid("true"); //plt::ylim(-1, 2000);
plt::plot(rms_g2o_error_ground4,  {{"label", "ground4"}}); plt::legend(); plt::grid("true"); //plt::ylim(-1, 2000);
plt::plot(rms_g2o_error_ground5,  {{"label", "ground5"}}); plt::legend(); plt::grid("true"); //plt::ylim(-1, 2000);

plt::tight_layout();

// plt::plot(rms_g2o_error_ground1,  {{"label", "wall1"}}); plt::plot(rms_g2o_error_ground2,  {{"label", "ground2"}}); 
// plt::plot(rms_g2o_error_wall1,  {{"label", "wall1"}});     plt::plot(rms_g2o_error_wall2,  {{"label", "wall2"}}); 
// plt::plot(rms_g2o_error_wall3,  {{"label", "wall3"}});      plt::plot(rms_g2o_error_wall4,  {{"label", "wall4"}}); 
// plt::plot(rms_g2o_error_wall5,  {{"label", "wall5"}}); plt::legend(); 

plt::figure();
plt::suptitle("Distance ground");
plt::subplot(4, 2, 1); plt::plot(distances_ground1,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground1,  {{"label", "After"}}); plt::title("ground1");  plt::legend(); plt::grid("true");
plt::subplot(4, 2, 2); plt::plot(distances_ground2,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground2,  {{"label", "After"}}); plt::title("ground2"); plt::legend(); plt::grid("true");
plt::subplot(4, 2, 3); plt::plot(distances_ground3,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground3,  {{"label", "After"}}); plt::title("ground3"); plt::legend(); plt::grid("true");
plt::subplot(4,2,4); plt::plot(distances_ground4,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground4,  {{"label", "After"}}); plt::title("ground4"); plt::legend(); plt::grid("true");
plt::subplot(4,2,5); plt::plot(distances_ground5,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground5,  {{"label", "After"}}); plt::title("ground5"); plt::legend(); plt::grid("true");



std::cout<<" ================= Distance ==================== " <<std::endl;
plt::figure();
plt::suptitle("Distance");
plt::subplot(4, 2, 1); plt::plot(distances_ground1,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground1,  {{"label", "After"}}); plt::title("ground");  plt::legend(); plt::grid("true");
plt::subplot(4, 2, 2); plt::plot(distances_ground2,  {{"label", "Before"}}); plt::plot(after_opti_distances_ground2,  {{"label", "After"}}); plt::title("ground"); plt::legend(); plt::grid("true");
plt::subplot(4, 2, 3); plt::plot(distances_wall1,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall1,  {{"label", "After"}}); plt::title("wall1"); plt::legend(); plt::grid("true");
plt::subplot(4,2,4); plt::plot(distances_wall2,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall2,  {{"label", "After"}}); plt::title("wall2"); plt::legend(); plt::grid("true");
plt::subplot(4,2,5); plt::plot(distances_wall3,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall3,  {{"label", "After"}}); plt::title("wall3"); plt::legend(); plt::grid("true");
plt::subplot(4,2,6); plt::plot(distances_wall4,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall4,  {{"label", "After"}}); plt::title("wall4"); plt::legend(); plt::grid("true");
plt::subplot(4,2,7); plt::plot(distances_wall5,  {{"label", "Before"}}); plt::plot(after_opti_distances_wall5,  {{"label", "After"}}); plt::title("wall5"); plt::legend(); plt::grid("true");
// plt::subplot(4,2,8); plt::plot(rms_distances_all_plane,  {{"label", "Before"}}); plt::plot(after_opti_rms_distances_all_plane,  {{"label", "After"}}); plt::title("Sum");
plt::tight_layout();

std::vector<double> node_g1(edges_ground1.size(), node_ground1);
std::vector<double> node_g2(edges_ground2.size(), node_ground2);
std::vector<double> node_g3(edges_ground3.size(), node_ground3);
std::vector<double> node_w1(edges_wall1.size(), node_wall1);
std::vector<double> node_w2(edges_wall2.size(), node_wall2);
std::vector<double> node_w3(edges_wall3.size(), node_wall3);
std::vector<double> node_w4(edges_wall4.size(), node_wall4);
std::vector<double> node_w5(edges_wall5.size(), node_wall5);

std::cout<<" ================= Edges ==================== " <<std::endl;

// plt::figure();
// plt::suptitle("Edges");
// plt::subplot(4, 2, 1); plt::plot(edges_ground1, node_g1,  {{"label", "ground1"}, {"marker", "o"}, {"linestyle", "--"}} );  plt::legend(); plt::grid("true"); plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
// plt::subplot(4, 2, 2); plt::plot(edges_ground2, node_g2, {{"label", "ground2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true"); plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
// plt::subplot(4, 2, 3); plt::plot(edges_ground3, node_g3, {{"label", "ground3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true"); plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
// plt::subplot(4, 2, 4); plt::plot(edges_wall1, node_w1, {{"label", "wall1"}, {"marker", "o"}, {"linestyle", "--"}});  plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
// plt::subplot(4,2,5); plt::plot(edges_wall2,  node_w2,{{"label", "wall2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
// plt::subplot(4,2,6); plt::plot(edges_wall3, node_w3,  {{"label", "wall3"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
// plt::subplot(4,2,7); plt::plot(edges_wall4, node_w4,  {{"label", "wall4"}, {"marker", "o"}, {"linestyle", "--"}});   plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
// plt::subplot(4,2,8); plt::plot(edges_wall5,  node_w5, {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}});  plt::legend(); plt::grid("true");plt::xlabel("pose nodes for each scan line"); plt::ylabel("plane node");
// plt::subplot(4,2,8); plt::plot(edges_ground1, node_g1,  {{"label", "ground1"}, {"marker", "o"}, {"linestyle", "--"}}); plt::plot(edges_ground2, node_g2,  {{"label", "ground2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::grid("true");
// plt::plot(edges_wall1, node_w1, {{"label", "wall1"}, {"marker", "o"}, {"linestyle", "--"}});     plt::plot(edges_wall2, node_w2,  {{"label", "wall2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::grid("true");
// plt::plot(edges_wall3, node_w3,  {{"label", "wall3"}, {"marker", "o"}, {"linestyle", "--"}});      plt::plot(edges_wall4, node_w4,  {{"label", "wall4"}, {"marker", "o"}, {"linestyle", "--"}}); plt::grid("true");
// plt::plot(edges_wall5, node_w5,  {{"label", "wall5"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); 

std::cout<<" ================= End ==================== " <<std::endl;



plt::show();

do 
{
std::cout << '\n' << "Press a key to continue...";
} while (std::cin.get() != '\n');

plt::close();

return 0;
}




