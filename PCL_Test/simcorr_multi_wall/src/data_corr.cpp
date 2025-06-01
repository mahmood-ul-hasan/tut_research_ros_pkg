#include<pre_processing.hpp>
#include<key_index.hpp>
#include<graph_optimization.hpp>


using namespace std::chrono_literals;
//using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;

pcl::PointCloud<PointT>::Ptr plane_points_ground(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr filterred_data (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall1 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall2 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall3 (new pcl::PointCloud<PointT>);

Eigen::VectorXf coeffs_ground;
Eigen::VectorXf coeffs_wall1;
Eigen::VectorXf coeffs_wall2;
Eigen::VectorXf coeffs_wall3;

key_index filterred_idx;
key_index plane_idx_ground;
key_index plane_idx_wall1;
key_index plane_idx_wall2;
key_index plane_idx_wall3;

std::vector<double> rss_distances_ground;
std::vector<double> rss_distances_wall1;
std::vector<double> rss_distances_wall2;
std::vector<double> rss_distances_wall3;

int main (int argc, char** argv)
{
  std::vector<int> inlier_idx_ground;
  std::vector<int> inlier_index_wall1;
  std::vector<int> inlier_index_wall2;
  std::vector<int> inlier_index_wall3;
     
  std::string pcd_file = "original_pcd.pcd";
  // initialize PointClouds and load point cloud
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  load_pcd_file(pcd_file, cloud);
  
  key_index original_index;
  original_index = gen_index_original_data(cloud); //point cloud index for the original cloud
  std::vector<int> indices;
  remove_nan_data(filterred_data, cloud, indices);  //remove the NaN data from the original cloud
  
  filterred_idx = gen_index_general(original_index, indices);  //re-map the index after removal of NaN data
  //voxel_grid_filter(filterred_cloud, filterred_data);
  
  std::vector<int> inliers_idxs[4];
  std::vector<int> outliers_idxs[4];
  std::vector<float>coefficients[4];
  fit_plane(pcd_file,coefficients,inliers_idxs, outliers_idxs);  //fit the plane to the cloud data 
  coeffs_ground = Eigen::Vector4f(coefficients[0][0], coefficients[0][1], coefficients[0][2], coefficients[0][3]);
  coeffs_wall1 = Eigen::Vector4f(coefficients[1][0], coefficients[1][1], coefficients[1][2], coefficients[1][3]);
  coeffs_wall2 = Eigen::Vector4f(coefficients[2][0], coefficients[2][1], coefficients[2][2], coefficients[2][3]);
  coeffs_wall3 = Eigen::Vector4f(coefficients[3][0], coefficients[3][1], coefficients[3][2], coefficients[3][3]);
  inlier_idx_ground = inliers_idxs[0];
  inlier_index_wall1 = inliers_idxs[1];
  inlier_index_wall2 = inliers_idxs[2];
  inlier_index_wall3 = inliers_idxs[3];
    
  //map the indices
  plane_idx_ground = gen_index_general(filterred_idx, inlier_idx_ground);  //re-map the index to the fitted inlier points
  //map the plane index for wall1
  key_index level1_idx;
  level1_idx = gen_index_general(filterred_idx, outliers_idxs[0]);
  plane_idx_wall1 = gen_index_general(level1_idx, inlier_index_wall1);
  //map the plane index for wall2
  key_index level2_idx;
  level2_idx = gen_index_general(level1_idx, outliers_idxs[1]);
  plane_idx_wall2 = gen_index_general(level2_idx, inlier_index_wall2);
  //map the plane index for wall3
  key_index level3_idx;
  level3_idx = gen_index_general(level2_idx, outliers_idxs[2]);
  plane_idx_wall3 = gen_index_general(level3_idx, inlier_index_wall3);

  
  std::vector<double> distances_ground;
  compute_distance(plane_points_ground, coeffs_ground, distances_ground);  //compute the distance to the plane from the fitted data
  //compute the frame_id wise rss distance to the plane
  compute_rss_distance(rss_distances_ground, distances_ground, plane_idx_ground); 
  //compute distance to wall from fitted points ans also compute the frame_wise rss distances
  std::vector<double>distances_wall1;
  compute_distance(plane_points_wall1, coeffs_wall1, distances_wall1);
  compute_rss_distance(rss_distances_wall1, distances_wall1, plane_idx_wall1);
  std::vector<double>distances_wall2;
  compute_distance(plane_points_wall2, coeffs_wall2, distances_wall2);
  compute_rss_distance(rss_distances_wall2, distances_wall2, plane_idx_wall2);
  std::vector<double>distances_wall3;
  compute_distance(plane_points_wall3, coeffs_wall3, distances_wall3);
  compute_rss_distance(rss_distances_wall3, distances_wall3, plane_idx_wall3);
  
  /*for(int i:original_index.frame_id){
    std::cout<<filterred_idx.frame_id[i]<<"\t"<<filterred_idx.min_index[i]<<"\t"<<filterred_idx.max_index[i]<<"\t";
    std::cout<<level3_idx.frame_id[i]<<"\t"<<level3_idx.min_index[i]<<"\t"<<level3_idx.max_index[i]<<"\t";
    std::cout<<plane_idx_wall3.frame_id[i]<<"\t"<<plane_idx_wall3.min_index[i]<<"\t"<<plane_idx_wall3.max_index[i]<<"\t";//<<std::endl;
    std::cout<<rss_distances_wall3[i]<<std::endl;
  }*/
  
  //display some distance data  
  //for(auto i:distances_wall3)
    //std::cout<<i<<std::endl;
  //std::cout<<"Data size: "<<distances.size();
 
  graph_construction();
  graph_optimization();
  //display_pose();
  modified_pcd_file(); 

  double mean, sd;
  calculate_mean_sd_distances(distances_ground, mean, sd);
  std::cout<<"Mean and SD distances for ground plane before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  calculate_mean_sd_distances(distances_wall1, mean, sd);
  std::cout<<"Mean and SD distances for wall plane-1 before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  calculate_mean_sd_distances(distances_wall2, mean, sd);
  std::cout<<"Mean and SD distances for wall plane-2 before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  calculate_mean_sd_distances(distances_wall3, mean, sd);
  std::cout<<"Mean and SD distances for wall plane-3 before optimization are: "<<mean<<"\t"<<sd<<std::endl;
  display_RSSE(); 
  return 0;
}





