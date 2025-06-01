#ifndef GRAPH_OPTIMIZATION_HPP
#define GRAPH_OPTIMIZATION_HPP

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


typedef pcl::PointXYZI PointT;

std::vector<double> x;
std::vector<double> y;
std::vector<double> z;
std::vector<double> Roll;
std::vector<double> Pitch;
std::vector<double> Yaw;

std::vector<double> edges_ground1;
std::vector<double> edges_ground2;
std::vector<double> edges_ground3;
std::vector<double> edges_ground4;
std::vector<double> edges_wall1;
std::vector<double> edges_wall2;
std::vector<double> edges_wall3;
std::vector<double> edges_wall4;
std::vector<double> edges_wall5;




int node_ground1;
int node_ground2;
int node_ground3;
int node_ground4;
int node_wall1;
int node_wall2;
int node_wall3;
int node_wall4;
int node_wall5;


void graph_construction(Eigen::VectorXf &floor_coeffs);
void graph_optimization(int num_iteration);
void display_pose(ros::Publisher odom_pub);
pcl::PointCloud<PointT>::Ptr modified_pcd_file();
void extract_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, int vertexID);
void merge_point_cloud(pcl::PointCloud<PointT>::Ptr transformed_cloud, pcl::PointCloud<PointT>::Ptr merge_cloud, int vertexID);
visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp);
pcl::PointCloud<pcl::PointXYZ>::Ptr smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif // GRAPH_OPTIMIZATION_HPP
