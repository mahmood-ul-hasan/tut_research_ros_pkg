#ifndef PRE_PROCESSING_HPP
#define PRE_PROCESSING_HPP
#include <iostream>
#include <math.h>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>


#include<key_index.hpp>


typedef pcl::PointXYZI PointT;


/*Global variables*/
extern pcl::PointCloud<PointT>::Ptr plane_points;
extern pcl::PointCloud<PointT>::Ptr filterred_data;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall;
extern Eigen::VectorXf coeffs;
extern key_index plane_idx;
extern key_index filterred_idx;
extern std::vector<double> rms_distances;
extern std::vector<double> rms_distances_wall;
extern Eigen::VectorXf coeffs_wall;
extern key_index plane_idx_wall;

void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud);
void remove_nan_data(pcl::PointCloud<PointT>::Ptr filterred_data, pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &indices);
void voxel_grid_filter(pcl::PointCloud<PointT>::Ptr filterred_cloud, pcl::PointCloud<PointT>::Ptr cloud);
void fit_plane(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<int> &inliers);
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<double> &distances);
void compute_rms_distance(std::vector<double> &rms_distances, std::vector<double> &distances, key_index inliers_idx);
double ground_plane_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
//void extract_frame_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, int vertexID);
void extract_frame_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr plane_cloud, key_index plane_index, int vertexID);
void trnasform_point_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr transformed_cloud, Eigen::Isometry3d transform_mat);
void load_wall_info(std::vector<int> &roi_wall_idx, std::vector<int> &inlier_wall_idx, Eigen::VectorXf &coeffs_wall);
double wall_plane_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);


#endif // PRE_PROCESSING_HPP
