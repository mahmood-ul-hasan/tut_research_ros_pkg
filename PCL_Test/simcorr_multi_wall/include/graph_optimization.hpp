#ifndef GRAPH_OPTIMIZATION_HPP
#define GRAPH_OPTIMIZATION_HPP

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include<key_index.hpp>

typedef pcl::PointXYZI PointT;

void graph_construction();
void graph_optimization();
void display_pose();
void modified_pcd_file();
void extract_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, int vertexID);
void merge_point_cloud(pcl::PointCloud<PointT>::Ptr transformed_cloud, pcl::PointCloud<PointT>::Ptr merge_cloud, int vertexID);
double calc_globalRSS_after_optimization(pcl::PointCloud<PointT>::Ptr plane_pts, key_index plane_index, Eigen::VectorXf coeff, std::vector<double> &distances);

#endif // GRAPH_OPTIMIZATION_HPP
