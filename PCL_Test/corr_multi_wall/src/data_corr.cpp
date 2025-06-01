#include<pre_processing.hpp>
#include<key_index.hpp>
#include<graph_optimization.hpp>


using namespace std::chrono_literals;
//using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;

pcl::PointCloud<PointT>::Ptr plane_points(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr filterred_data (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall1 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall2 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall3 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall4 (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall5 (new pcl::PointCloud<PointT>);

Eigen::VectorXf coeffs;
key_index plane_idx;
key_index filterred_idx;
Eigen::VectorXf coeffs_wall1(4);
Eigen::VectorXf coeffs_wall2(4);
Eigen::VectorXf coeffs_wall3(4);
Eigen::VectorXf coeffs_wall4(4);
Eigen::VectorXf coeffs_wall5(4);

key_index plane_idx_wall1;
key_index plane_idx_wall2;
key_index plane_idx_wall3;
key_index plane_idx_wall4;
key_index plane_idx_wall5;


std::vector<double> rms_distances;
std::vector<double> rms_distances_wall1;
std::vector<double> rms_distances_wall2;
std::vector<double> rms_distances_wall3;
std::vector<double> rms_distances_wall4;
std::vector<double> rms_distances_wall5;

int main (int argc, char** argv)
{
  std::vector<int> roi_index_wall1;
  std::vector<int> inlier_index_wall1;
  std::vector<int> roi_index_wall2;
  std::vector<int> inlier_index_wall2;
  std::vector<int> roi_index_wall3;
  std::vector<int> inlier_index_wall3;
  std::vector<int> roi_index_wall4;
  std::vector<int> inlier_index_wall4;
  std::vector<int> roi_index_wall5;
  std::vector<int> inlier_index_wall5;
  key_index filterred_idx_wall1;
  key_index filterred_idx_wall2;
  key_index filterred_idx_wall3;
  key_index filterred_idx_wall4;
  key_index filterred_idx_wall5;
  
  std::string pcd_file = "original_data.pcd";
  // initialize PointClouds and load point cloud
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  load_pcd_file(pcd_file, cloud);
  
  //load wall infomation file
  std::string wall_info_file;
  wall_info_file = "wall_seg1.txt";
  load_wall_info(wall_info_file, roi_index_wall1,inlier_index_wall1,coeffs_wall1);
  wall_info_file = "wall_seg2.txt";
  load_wall_info(wall_info_file, roi_index_wall2,inlier_index_wall2,coeffs_wall2);
  wall_info_file = "wall_seg3.txt";
  load_wall_info(wall_info_file, roi_index_wall3,inlier_index_wall3,coeffs_wall3);
  wall_info_file = "wall_seg4.txt";
  load_wall_info(wall_info_file, roi_index_wall4,inlier_index_wall4,coeffs_wall4);
  wall_info_file = "wall_seg5.txt";
  load_wall_info(wall_info_file, roi_index_wall5,inlier_index_wall5,coeffs_wall5);

  //load fitted wall points from wall_seg_cloud.pcd file
  pcd_file = "wall_seg_cloud1.pcd";
  load_pcd_file(pcd_file,plane_points_wall1);
  pcd_file = "wall_seg_cloud2.pcd";
  load_pcd_file(pcd_file,plane_points_wall2);
  pcd_file = "wall_seg_cloud3.pcd";
  load_pcd_file(pcd_file,plane_points_wall3);
  pcd_file = "wall_seg_cloud4.pcd";
  load_pcd_file(pcd_file,plane_points_wall4);
  pcd_file = "wall_seg_cloud5.pcd";
  load_pcd_file(pcd_file,plane_points_wall5);

  key_index original_index;
  original_index = gen_index_original_data(cloud); //point cloud index for the original cloud
  std::vector<int> indices;
  remove_nan_data(filterred_data, cloud, indices);  //remove the NaN data from the original cloud
  
  filterred_idx = gen_index_general(original_index, indices);  //re-map the index after removal of NaN data
  //voxel_grid_filter(filterred_cloud, filterred_data);
  
  std::vector<int> inliers_idx;
  fit_plane(filterred_data,plane_points,coeffs,inliers_idx);  //fit the plane to the cloud data 
  std::vector<double> distances;
  compute_distance(plane_points, coeffs, distances);  //compute the distance to the plane from the fitted data
  
  plane_idx = gen_index_general(filterred_idx, inliers_idx);  //re-map the index to the fitted inlier points

  //compute the frame_id wise rms distance to the plane
  compute_rms_distance(rms_distances, distances, plane_idx); 

  //map between the roi_index_wall and filterred index, and plane_index_wall
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

  //compute distance to wall from fitted points ans also compute the frame_wise rms distances
  std::vector<double>distances_wall1;
  compute_distance(plane_points_wall1, coeffs_wall1, distances_wall1);
  compute_rms_distance(rms_distances_wall1, distances_wall1, plane_idx_wall1);
  std::vector<double>distances_wall2;
  compute_distance(plane_points_wall2, coeffs_wall2, distances_wall2);
  compute_rms_distance(rms_distances_wall2, distances_wall2, plane_idx_wall2);
  std::vector<double>distances_wall3;
  compute_distance(plane_points_wall3, coeffs_wall3, distances_wall3);
  compute_rms_distance(rms_distances_wall3, distances_wall3, plane_idx_wall3);
  std::vector<double>distances_wall4;
  compute_distance(plane_points_wall4, coeffs_wall4, distances_wall4);
  compute_rms_distance(rms_distances_wall4, distances_wall4, plane_idx_wall4);
  std::vector<double>distances_wall5;
  compute_distance(plane_points_wall5, coeffs_wall5, distances_wall5);
  compute_rms_distance(rms_distances_wall5, distances_wall5, plane_idx_wall5);


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
  
  graph_construction(coeffs);
  graph_optimization();
  //display_pose();
  modified_pcd_file(); 
  return 0;
}





