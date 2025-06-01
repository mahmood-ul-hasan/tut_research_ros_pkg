#ifndef PRE_PROCESSING_HPP
#define PRE_PROCESSING_HPP
#include <iostream>
#include <math.h>
#include <thread>
#include <hdl_graph_slam/graph_slam.hpp>
#include <g2o/types/slam3d/vertex_se3.h>

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
#include <pcl/common/angles.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"

#include<key_index.hpp>

using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;


/*Global variables*/
extern std::unique_ptr<GraphSLAM> graph_slam;
extern std::vector<g2o::VertexSE3*> node_array;

extern pcl::PointCloud<PointT>::Ptr plane_points_ground;
extern pcl::PointCloud<PointT>::Ptr filterred_data;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall1;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall2;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall3;

extern Eigen::VectorXf coeffs_ground;
extern key_index plane_idx_ground;
extern key_index filterred_idx;

extern std::vector<double> rss_distances_ground;
extern std::vector<double> rss_distances_wall1;
extern std::vector<double> rss_distances_wall2;
extern std::vector<double> rss_distances_wall3;

extern Eigen::VectorXf coeffs_wall1;
extern Eigen::VectorXf coeffs_wall2;
extern Eigen::VectorXf coeffs_wall3;

extern key_index plane_idx_wall1;
extern key_index plane_idx_wall2;
extern key_index plane_idx_wall3;

void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud);
void remove_nan_data(pcl::PointCloud<PointT>::Ptr filterred_data, pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &indices);
void voxel_grid_filter(pcl::PointCloud<PointT>::Ptr filterred_cloud, pcl::PointCloud<PointT>::Ptr cloud);
void fit_plane(std::string pcd_file, std::vector<float> coeffs_set[3], std::vector<int> inliers_set[3], std::vector<int> outliers_set[3]);
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<double> &distances);
void compute_rss_distance(std::vector<double> &rss_distances, std::vector<double> &distances, key_index inliers_idx);
double ground_plane_rss_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
void extract_frame_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr plane_cloud, key_index plane_index, int vertexID);
void trnasform_point_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr transformed_cloud, Eigen::Isometry3d transform_mat);
double wall_plane1_rss_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double wall_plane2_rss_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double wall_plane3_rss_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
void calculate_mean_sd_distances(std::vector<double> dist, double &mean, double &sd);
void display_RSSE();


#endif // PRE_PROCESSING_HPP
