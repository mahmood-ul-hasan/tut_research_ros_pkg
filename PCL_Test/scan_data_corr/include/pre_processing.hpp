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
extern Eigen::VectorXf coeffs;
extern key_index plane_idx;
extern key_index filterred_idx;
extern std::vector<double> rms_distances;

void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud);
void remove_nan_data(pcl::PointCloud<PointT>::Ptr filterred_data, pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &indices);
void voxel_grid_filter(pcl::PointCloud<PointT>::Ptr filterred_cloud, pcl::PointCloud<PointT>::Ptr cloud);
void fit_plane(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<int> &inliers);
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<double> &distances);
void compute_rms_distance(std::vector<double> &rms_distances, std::vector<double> &distances, key_index inliers_idx);
double compute_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
void extract_frame_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, int vertexID);
void trnasform_point_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr transformed_cloud, Eigen::Isometry3d transform_mat);


#endif // PRE_PROCESSING_HPP
