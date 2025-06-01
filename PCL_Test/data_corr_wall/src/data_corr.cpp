#include<pre_processing.hpp>
#include<key_index.hpp>
#include<graph_optimization.hpp>


using namespace std::chrono_literals;
//using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;

pcl::PointCloud<PointT>::Ptr plane_points(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr filterred_data (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr plane_points_wall (new pcl::PointCloud<PointT>);
Eigen::VectorXf coeffs;
key_index plane_idx;
key_index filterred_idx;
Eigen::VectorXf coeffs_wall(4);
key_index plane_idx_wall;
key_index filterred_idx_wall;

std::vector<double> rms_distances;
std::vector<double> rms_distances_wall;

int main (int argc, char** argv)
{
  std::vector<int> roi_index_wall;
  std::vector<int> inlier_index_wall; 
  std::string pcd_file = "original_data.pcd";
  // initialize PointClouds and load point cloud
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  load_pcd_file(pcd_file, cloud);
  //load wall infomation file
  load_wall_info(roi_index_wall,inlier_index_wall,coeffs_wall);
  //load fitted wall points from wall_seg_cloud.pcd file
  pcd_file = "wall_seg_cloud.pcd";
  load_pcd_file(pcd_file,plane_points_wall);

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
  filterred_idx_wall = gen_index_general(filterred_idx, roi_index_wall);
  plane_idx_wall = gen_index_general(filterred_idx_wall, inlier_index_wall);
  //compute distance to wall from fitted points ans also compute the frame_wise rms distances
  std::vector<double>distances_wall;
  compute_distance(plane_points_wall, coeffs_wall, distances_wall);
  compute_rms_distance(rms_distances_wall, distances_wall, plane_idx_wall);

  /*for(int i:original_index.frame_id){
    std::cout<<filterred_idx.frame_id[i]<<"  "<<filterred_idx.min_index[i]<<"  "<<filterred_idx.max_index[i]<<"     ";
    std::cout<<filterred_idx_wall.frame_id[i]<<"  "<<filterred_idx_wall.min_index[i]<<"  "<<filterred_idx_wall.max_index[i]<<"   ";
    std::cout<<plane_idx_wall.frame_id[i]<<"  "<<plane_idx_wall.min_index[i]<<"  "<<plane_idx_wall.max_index[i]<<std::endl;
    std::cout<<rms_distances[i]<<std::endl;
  }*/
  
  //display some distance data  
  //for(int i:plane_idx_wall.frame_id)
    //std::cout<<i<<" "<<rms_distances_wall[i]<<std::endl;
  //std::cout<<"Data size: "<<distances.size();
  
  graph_construction(coeffs);
  graph_optimization();
  display_pose();
  modified_pcd_file(); 
  return 0;
}





