#include<pre_processing.hpp>
#include<key_index.hpp>
#include<graph_optimization.hpp>


using namespace std::chrono_literals;
//using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;

pcl::PointCloud<PointT>::Ptr plane_points(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr filterred_data (new pcl::PointCloud<PointT>);
Eigen::VectorXf coeffs;
key_index plane_idx;
key_index filterred_idx;

std::vector<double> rms_distances;


int main (int argc, char** argv)
{
  
  std::string pcd_file = "original_pcd.pcd";
  // initialize PointClouds
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  
  //pcl::PointCloud<PointT>::Ptr filterred_cloud (new pcl::PointCloud<PointT>);
  
  load_pcd_file(pcd_file, cloud);
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

 /* for(int i:original_index.frame_id){
    std::cout<<original_index.frame_id[i]<<"  "<<original_index.min_index[i]<<"  "<<original_index.max_index[i]<<"     ";
    std::cout<<filterred_idx.frame_id[i]<<"  "<<filterred_idx.min_index[i]<<"  "<<filterred_idx.max_index[i]<<"   ";
    std::cout<<plane_idx.frame_id[i]<<"  "<<plane_idx.min_index[i]<<"  "<<plane_idx.max_index[i]<<"   ";
    std::cout<<rms_distances[i]<<std::endl;
  }*/

  //display some distance data  
  //for(int i=0;i<20000;i++)
    //std::cout<<"Distances: "<<distances[i]<<std::endl;
  //std::cout<<"Data size: "<<distances.size();
  
   graph_construction(coeffs);
   graph_optimization();
   display_pose();
   modified_pcd_file();
   //Calculate the Global RMS Distances before and after optimization
   double global_rss = 0.0;
   for(double rss:rms_distances){
     global_rss += rss;
   }
   std::cout<<"The global RSS before optimization is: "<<global_rss<<std::endl;
   global_rss = calc_globalRSS_after_optimization(plane_points, plane_idx, coeffs);
   std::cout<<"The global RSS after optimization is: "<<global_rss<<std::endl;

  return 0;
}





