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




#include<key_index.hpp>

using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;


/*Global variables*/
extern std::unique_ptr<GraphSLAM> graph_slam;
extern std::vector<g2o::VertexSE3*> node_array;
extern std::vector<double> dist_array;
extern std::vector<int> vertexID_array;


extern pcl::PointCloud<PointT>::Ptr plane_points_ground1;
extern pcl::PointCloud<PointT>::Ptr plane_points_ground2;
extern pcl::PointCloud<PointT>::Ptr plane_points_ground3;
extern pcl::PointCloud<PointT>::Ptr plane_points_ground4;
extern pcl::PointCloud<PointT>::Ptr plane_points_ground5;
extern pcl::PointCloud<PointT>::Ptr filterred_data;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall1;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall2;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall3;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall4;
extern pcl::PointCloud<PointT>::Ptr plane_points_wall5;

extern std::vector<double> rms_g2o_error_ground;
extern std::vector<double> rms_g2o_error_ground1;
extern std::vector<double> rms_g2o_error_ground2;
extern std::vector<double> rms_g2o_error_ground3;
extern std::vector<double> rms_g2o_error_ground4;
extern std::vector<double> rms_g2o_error_ground5;
extern std::vector<double> rms_g2o_error_wall1;
extern std::vector<double> rms_g2o_error_wall2;
extern std::vector<double> rms_g2o_error_wall3;
extern std::vector<double> rms_g2o_error_wall4;
extern std::vector<double> rms_g2o_error_wall5;


extern key_index filterred_idx;
extern std::vector<double> rms_distances_ground1;
extern std::vector<double> rms_distances_ground2;
extern std::vector<double> rms_distances_ground3;
extern std::vector<double> rms_distances_ground4;
extern std::vector<double> rms_distances_ground5;
extern std::vector<double> rms_distances_wall1;
extern std::vector<double> rms_distances_wall2;
extern std::vector<double> rms_distances_wall3;
extern std::vector<double> rms_distances_wall4;
extern std::vector<double> rms_distances_wall5;

extern std::vector<double> after_opti_rms_distances_ground1;
extern std::vector<double> after_opti_rms_distances_ground2;
extern std::vector<double> after_opti_rms_distances_ground3;
extern std::vector<double> after_opti_rms_distances_ground4;
extern std::vector<double> after_opti_rms_distances_ground5;
extern std::vector<double> after_opti_rms_distances_wall1;
extern std::vector<double> after_opti_rms_distances_wall2;
extern std::vector<double> after_opti_rms_distances_wall3;
extern std::vector<double> after_opti_rms_distances_wall4;
extern std::vector<double> after_opti_rms_distances_wall5;

extern std::vector<double> after_opti_distances_ground1;
extern std::vector<double> after_opti_distances_ground2;
extern std::vector<double> after_opti_distances_ground3;
extern std::vector<double> after_opti_distances_ground4;
extern std::vector<double> after_opti_distances_ground5;
extern std::vector<double> after_opti_distances_wall1;
extern std::vector<double> after_opti_distances_wall2;
extern std::vector<double> after_opti_distances_wall3;
extern std::vector<double> after_opti_distances_wall4;
extern std::vector<double> after_opti_distances_wall5;

extern Eigen::VectorXf coeffs_ground;
extern Eigen::VectorXf coeffs_ground1;
extern Eigen::VectorXf coeffs_ground2;
extern Eigen::VectorXf coeffs_ground3;
extern Eigen::VectorXf coeffs_ground4;
extern Eigen::VectorXf coeffs_ground5;
extern Eigen::VectorXf coeffs_wall1;
extern Eigen::VectorXf coeffs_wall2;
extern Eigen::VectorXf coeffs_wall3;
extern Eigen::VectorXf coeffs_wall4;
extern Eigen::VectorXf coeffs_wall5;

extern key_index plane_idx_ground1;
extern key_index plane_idx_ground2;
extern key_index plane_idx_ground3;
extern key_index plane_idx_ground4;
extern key_index plane_idx_ground5;
extern key_index plane_idx_wall1;
extern key_index plane_idx_wall2;
extern key_index plane_idx_wall3;
extern key_index plane_idx_wall4;
extern key_index plane_idx_wall5;

extern int number_of_wall_planes;
extern int number_of_ground_planes;

void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud);
void remove_nan_data(pcl::PointCloud<PointT>::Ptr filterred_data, pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &indices);
void voxel_grid_filter(pcl::PointCloud<PointT>::Ptr filterred_cloud, pcl::PointCloud<PointT>::Ptr cloud);
void fit_plane(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<int> &inliers);
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<double> &distances);
// void compute_rms_distance(std::vector<double> &rms_distances, std::vector<double> &distances, std::vector<int> indices, key_index inliers_idx);
void compute_rms_distance(std::vector<double> &rms_distances, std::vector<double> &distances,  key_index inliers_idx);
double ground_plane_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double ground_plane2_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double ground_plane3_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double ground_plane4_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
void extract_frame_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr plane_cloud, key_index plane_index, int vertexID);
void trnasform_point_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr transformed_cloud, Eigen::Isometry3d transform_mat);
void load_wall_info(std::string file, std::vector<int> &roi_wall_idx, std::vector<int> &inlier_wall_idx, Eigen::VectorXf &coeffs_wall);
double wall_plane1_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double wall_plane2_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double wall_plane3_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double wall_plane4_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);
double wall_plane5_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID);

void display_RSSE();
void calculate_mean_sd_distances(std::vector<double> dist, double &mean, double &sd);
double calc_globalRSS_after_optimization(pcl::PointCloud<PointT>::Ptr plane_pts, key_index plane_index, Eigen::VectorXf coeff, std::vector<double> &distances, std::vector<double> &frame_wise_rss);



#endif // PRE_PROCESSING_HPP
