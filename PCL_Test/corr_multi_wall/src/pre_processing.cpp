#include<pre_processing.hpp>
#include<key_index.hpp>
#include <fstream>
#include <string>

using namespace std::chrono_literals;
typedef pcl::PointXYZI PointT;

//load the pcd file and store the point cloud data to the cloud variable
void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud)
{
  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the pcd file\n");
    std::exit(-1);
  }
  std::cout << "Loaded "
            << cloud->width <<"x"<< cloud->height
            << " data points from the pcd file"
            << std::endl;
} 

//remove the NaN data from the original point cloud data
void remove_nan_data(pcl::PointCloud<PointT>::Ptr filterred_data, pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &indices)
{
  pcl::removeNaNFromPointCloud(*cloud, *filterred_data, indices);
  std::cout << "Cloud size after NaN removal: " << filterred_data->points.size () << std::endl;
}

//create the voxel grid filter
void voxel_grid_filter(pcl::PointCloud<PointT>::Ptr filterred_cloud, pcl::PointCloud<PointT>::Ptr cloud)
{
   // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*filterred_cloud);

  std::cerr << "PointCloud after filtering: " << filterred_cloud->width * filterred_cloud->height 
       << " data points (" << pcl::getFieldsList (*filterred_cloud) << ").";
}
  
//fit the plane to the cloud data
void fit_plane(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<int> &inliers)
{

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT> (cloud));
  pcl::RandomSampleConsensus<PointT> ransac (model_p);
  ransac.setDistanceThreshold (0.02);
  ransac.computeModel();
  ransac.getInliers(inliers);
      
  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud (*cloud, inliers, *final);
  pcl::io::savePCDFileASCII ("test_pcd.pcd", *final);
  std::cerr << "Saved " << final->points.size () << " data points to test_pcd.pcd." << std::endl;
  //get the model coefficients first
  ransac.getModelCoefficients(coeffs);
}

//compute the distance of the inliers data to the plane
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeff, std::vector<double> &distances)
{
  //Compute the distances to the model
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_inliers(new pcl::SampleConsensusModelPlane<PointT> (final));
  model_inliers->getDistancesToModel(coeff, distances);
}

//compute the frame_id wise average rms distance to the plane
void compute_rms_distance(std::vector<double> &rms_distances, std::vector<double> &distances, key_index inliers_idx)
{
  for(int i:inliers_idx.frame_id){
      double dist = 0;
      if((inliers_idx.min_index[i] == -1) && (inliers_idx.max_index[i] == -1))
        dist = 0;
      else{
        for(size_t j= inliers_idx.min_index[i]; j <= inliers_idx.max_index[i]; j++)
          dist += pow(distances[j],2);
      }
      rms_distances.push_back(sqrt(dist));
  }
}

//compute the frame wise rms error for ground plane in order to calculate the error vector
double ground_plane_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist;
  extract_frame_cloud(frame_cloud, plane_points, plane_idx, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs, frame_pts_distances);
  //compute the rms distance
  for(size_t i = 0; i<(plane_idx.max_index[vertexID-1] - plane_idx.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rms_dist = sqrt(dist);
  
  std::cout<<"F_ID:"<<vertexID<<" rms_dist diff ground plane:"<<rms_dist-rms_distances[vertexID-1]<<"\n";
  return rms_dist;
}


//compute the frame wise rms error for wall plane1 in order to calculate the error vector
double wall_plane1_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall;
  extract_frame_cloud(frame_cloud, plane_points_wall1, plane_idx_wall1, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_wall1, frame_pts_distances);
  //compute the rms distance
  for(size_t i = 0; i<(plane_idx_wall1.max_index[vertexID-1] - plane_idx_wall1.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rms_dist_wall = sqrt(dist);
  
  std::cout<<"F_ID Wall1:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall1[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//compute the frame wise rms error for wall plane2 in order to calculate the error vector
double wall_plane2_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall;
  extract_frame_cloud(frame_cloud, plane_points_wall2, plane_idx_wall2, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_wall2, frame_pts_distances);
  //compute the rms distance
  for(size_t i = 0; i<(plane_idx_wall2.max_index[vertexID-1] - plane_idx_wall2.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rms_dist_wall = sqrt(dist);
  
  std::cout<<"F_ID Wall2:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall2[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//compute the frame wise rms error for wall plane3 in order to calculate the error vector
double wall_plane3_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall;
  extract_frame_cloud(frame_cloud, plane_points_wall3, plane_idx_wall3, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_wall3, frame_pts_distances);
  //compute the rms distance
  for(size_t i = 0; i<(plane_idx_wall3.max_index[vertexID-1] - plane_idx_wall3.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rms_dist_wall = sqrt(dist);
  
  std::cout<<"F_ID Wall3:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall3[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//compute the frame wise rms error for wall plane4 in order to calculate the error vector
double wall_plane4_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall;
  extract_frame_cloud(frame_cloud, plane_points_wall4, plane_idx_wall4, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_wall4, frame_pts_distances);
  //compute the rms distance
  for(size_t i = 0; i<(plane_idx_wall4.max_index[vertexID-1] - plane_idx_wall4.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rms_dist_wall = sqrt(dist);
  
  std::cout<<"F_ID Wall4:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall4[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//compute the frame wise rms error for wall plane5 in order to calculate the error vector
double wall_plane5_rms_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall;
  extract_frame_cloud(frame_cloud, plane_points_wall5, plane_idx_wall5, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_wall5, frame_pts_distances);
  //compute the rms distance
  for(size_t i = 0; i<(plane_idx_wall5.max_index[vertexID-1] - plane_idx_wall5.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rms_dist_wall = sqrt(dist);
  
  std::cout<<"F_ID Wall5:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall5[vertexID-1]<<"\n";
  return rms_dist_wall;
}


//extract the frame points cloud from the fitted plane cloud
void extract_frame_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr plane_cloud, key_index plane_index, int vertexID)
{
  frame_cloud->width    = plane_index.max_index[vertexID-1] - plane_index.min_index[vertexID-1] + 1;
  frame_cloud->height   = 1;
  frame_cloud->is_dense = false;
  frame_cloud->points.resize (frame_cloud->width * frame_cloud->height);

  for (size_t i = 0; i < frame_cloud->points.size (); ++i){
    frame_cloud->points[i] = plane_cloud->points[plane_index.min_index[vertexID-1]+i];
  }

}

//transform the points cloud according to the transformation matrix
void trnasform_point_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr transformed_cloud, Eigen::Isometry3d transform_mat)
{
  Eigen::Isometry3f trans_mat;
  trans_mat = transform_mat.cast<float>();
  //std::cout<<"Trans_mat:"<<trans_mat.matrix()<<"\n";
  pcl::transformPointCloud (*frame_cloud, *transformed_cloud, trans_mat);
}

//load the text file for wall information
void load_wall_info(std::string file, std::vector<int> &roi_wall_idx, std::vector<int> &inlier_wall_idx, Eigen::VectorXf &coeffs_wall)
{
  ifstream wfile;
  std::string line;
  int size,data;
  wfile.open(file);
  if(!wfile.is_open()){
    std::cerr<<"unable to open wall_seg file\n";
    return;
  }
  std::getline(wfile,line);
  //std::cout<<line<<std::endl;
  std::getline(wfile,line);
  //std::cout<<line<<std::endl;
  wfile>>size;
  //std::cout<<size<<std::endl;
  std::getline(wfile,line);
  std::getline(wfile,line);
  //std::cout<<line<<std::endl;
  for(size_t i=0; i<size; i++){
    wfile>>data;
    roi_wall_idx.push_back(data);
  }
  //for(size_t i=0; i<size; i++)
    //std::cout<<" "<<roi_wall_idx[i];
  std::getline(wfile,line);
  std::getline(wfile,line);
  //std::cout<<line<<std::endl;
  wfile>>coeffs_wall(0)>>coeffs_wall(1)>>coeffs_wall(2)>>coeffs_wall(3);
  //std::cout<<coeffs_wall<<std::endl;
  std::getline(wfile,line);
  std::getline(wfile,line);
  //std::cout<<line<<std::endl;
  std::getline(wfile,line);
  //std::cout<<line<<std::endl;
  wfile>>size;
  //std::cout<<size<<std::endl;
  std::getline(wfile,line);
  std::getline(wfile,line);
  //std::cout<<line<<std::endl;
  for(size_t i=0; i<size; i++){
    wfile>>data;
    inlier_wall_idx.push_back(data);
  }
  //for(size_t i=0; i<size; i++)
    //std::cout<<" "<<inlier_wall_idx[i];
  wfile.close();
}






