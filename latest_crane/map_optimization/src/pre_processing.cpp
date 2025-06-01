#include <pre_processing.hpp>
#include <key_index.hpp>
#include <fstream>
#include <string>

using namespace std::chrono_literals;
typedef pcl::PointXYZI PointT;

std::vector<double> rms_g2o_error_ground;
std::vector<double> rms_g2o_error_ground1;
std::vector<double> rms_g2o_error_ground2;
std::vector<double> rms_g2o_error_ground3;
std::vector<double> rms_g2o_error_ground4;
std::vector<double> rms_g2o_error_ground5;

std::vector<double> rms_g2o_error_wall1;
std::vector<double> rms_g2o_error_wall2;
std::vector<double> rms_g2o_error_wall3;
std::vector<double> rms_g2o_error_wall4;
std::vector<double> rms_g2o_error_wall5;

std::vector<double> after_opti_rms_distances_ground1;
std::vector<double> after_opti_rms_distances_ground2;
std::vector<double> after_opti_rms_distances_ground3;
std::vector<double> after_opti_rms_distances_ground4;
std::vector<double> after_opti_rms_distances_ground5;
std::vector<double> after_opti_rms_distances_wall1;
std::vector<double> after_opti_rms_distances_wall2;
std::vector<double> after_opti_rms_distances_wall3;
std::vector<double> after_opti_rms_distances_wall4;
std::vector<double> after_opti_rms_distances_wall5;

std::vector<double> after_opti_distances_ground1;
std::vector<double> after_opti_distances_ground2;
std::vector<double> after_opti_distances_ground3;
std::vector<double> after_opti_distances_ground4;
std::vector<double> after_opti_distances_ground5;
std::vector<double> after_opti_distances_wall1;
std::vector<double> after_opti_distances_wall2;
std::vector<double> after_opti_distances_wall3;
std::vector<double> after_opti_distances_wall4;
std::vector<double> after_opti_distances_wall5;

//load the pcd file and store the point cloud data to the cloud variable
void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud)
{
  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read the pcd file\n");
    std::exit(-1);
  }
  std::cout << "Loaded "
            << cloud->width << "x" << cloud->height
            << " data points from the pcd file"
            << std::endl;
}

//remove the NaN data from the original point cloud data
void remove_nan_data(pcl::PointCloud<PointT>::Ptr filterred_data, pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &indices)
{
  std::cout << "Cloud size before NaN removal: " << cloud->points.size() << std::endl;
  pcl::removeNaNFromPointCloud(*cloud, *filterred_data, indices);
  std::cout << "Cloud size after NaN removal: " << filterred_data->points.size() << std::endl;
}

//create the voxel grid filter
void voxel_grid_filter(pcl::PointCloud<PointT>::Ptr filterred_cloud, pcl::PointCloud<PointT>::Ptr cloud)
{
  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*filterred_cloud);

  std::cerr << "PointCloud after filtering: " << filterred_cloud->width * filterred_cloud->height
            << " data points (" << pcl::getFieldsList(*filterred_cloud) << ").";
}

//fit the plane to the cloud data
// void fit_plane(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeffs, std::vector<int> &inliers)
// {

//   // created RandomSampleConsensus object and compute the appropriated model
//   pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(cloud));
//   pcl::RandomSampleConsensus<PointT> ransac(model_p);
//   //ransac.setDistanceThreshold (0.02);
//   ransac.setDistanceThreshold(0.001);
//   ransac.computeModel();
//   ransac.getInliers(inliers);

//   // copies all inliers of the model computed to another PointCloud
//   pcl::copyPointCloud(*cloud, inliers, *final);
//   pcl::io::savePCDFileASCII("test_pcd.pcd", *final);
//   std::cerr << "Saved " << final->points.size() << " data points to test_pcd.pcd." << std::endl;
//   //get the model coefficients first
//   ransac.getModelCoefficients(coeffs);
// }




//compute the distance of the inliers data to the plane
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeff, std::vector<double> &distances)
{
  //Compute the distances to the model
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_inliers(new pcl::SampleConsensusModelPlane<PointT>(final));
  model_inliers->getDistancesToModel(coeff, distances);
}

//compute the frame_id wise average rms distance to the plane
void compute_rms_distance(std::vector<double> &rms_distances, std::vector<double> &distances, key_index inliers_idx)
{
  for (int i : inliers_idx.frame_id)
  {
    double dist = 0;
    if ((inliers_idx.min_index[i] == -1) && (inliers_idx.max_index[i] == -1))
      dist = 0;
    else
    {
      for (size_t j = inliers_idx.min_index[i]; j <= inliers_idx.max_index[i]; j++)
        dist += pow(distances[j], 2);
    }
    rms_distances.push_back(dist);
  }
}

// compute the frame wise rms error for ground plane in order to calculate the error vector
// double ground_plane_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
// {
//   pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
//   pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
//   std::vector<double> frame_pts_distances;
//   double rms_dist = 0;
//   double dist1 = 0, dist2 = 0, dist3 = 0, dist4 = 0, dist5 = 0;

//   extract_frame_cloud(frame_cloud, plane_points_ground1, plane_idx_ground1, vertexID);
//   trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
//   compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
//   //compute the rms distance
//   for (size_t i = 0; i < (plane_idx_ground1.max_index[vertexID - 1] - plane_idx_ground1.min_index[vertexID - 1] + 1); i++)
//     dist1 += pow(frame_pts_distances[i], 2);

//   // rms_dist = sqrt(dist);

//   // std::cout << " plane_idx_ground2 " << std::endl;

//   if ((plane_idx_ground2.min_index[vertexID] != -1) && (plane_idx_ground2.max_index[vertexID] != -1)){
//   extract_frame_cloud(frame_cloud, plane_points_ground2, plane_idx_ground2, vertexID);
//   trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
//   compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
//   //compute the rms distance
//   for (size_t i = 0; i < (plane_idx_ground2.max_index[vertexID - 1] - plane_idx_ground2.min_index[vertexID - 1] + 1); i++)
//     dist2 += pow(frame_pts_distances[i], 2);
//   }

//   // std::cout << " plane_idx_ground3 " << std::endl;
//   if ((plane_idx_ground3.min_index[vertexID] != -1) && (plane_idx_ground3.max_index[vertexID] != -1)){
//   extract_frame_cloud(frame_cloud, plane_points_ground3, plane_idx_ground3, vertexID);
//   trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
//   compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
//   //compute the rms distance
//   for (size_t i = 0; i < (plane_idx_ground3.max_index[vertexID - 1] - plane_idx_ground3.min_index[vertexID - 1] + 1); i++)
//     dist3 += pow(frame_pts_distances[i], 2);
//   }

//     if ((plane_idx_ground4.min_index[vertexID] != -1) && (plane_idx_ground4.max_index[vertexID] != -1)){
//   extract_frame_cloud(frame_cloud, plane_points_ground4, plane_idx_ground4, vertexID);
//   trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
//   compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
//   //compute the rms distance
//   for (size_t i = 0; i < (plane_idx_ground4.max_index[vertexID - 1] - plane_idx_ground4.min_index[vertexID - 1] + 1); i++)
//     dist4 += pow(frame_pts_distances[i], 2);
//   }

//   // std::cout << " plane_idx_ground5 " << std::endl;

//   if ((plane_idx_ground5.min_index[vertexID] != -1) && (plane_idx_ground5.max_index[vertexID] != -1)){
//   extract_frame_cloud(frame_cloud, plane_points_ground5, plane_idx_ground5, vertexID);
//   trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
//   compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
//   //compute the rms distance
//   for (size_t i = 0; i < (plane_idx_ground5.max_index[vertexID - 1] - plane_idx_ground5.min_index[vertexID - 1] + 1); i++)
//     dist5 += pow(frame_pts_distances[i], 2);
//   }

//   rms_dist = dist1+dist2+dist3+dist4+dist5;
  
//   std::cout << "rms_dist " << dist1 << " "<< dist2<< " " << dist3 <<" "<< dist4 <<" "<< dist5 <<" "<< rms_dist<<std::endl;

//     // std::cout<<"node_array: " << vertexID << "\n"<<  node_array[vertexID-1]->estimate().rotation() << std::endl;
//     rms_g2o_error_ground1.push_back(dist1);
//     rms_g2o_error_ground2.push_back(dist2);
//     rms_g2o_error_ground3.push_back(dist3);
//     rms_g2o_error_ground4.push_back(dist4);
//     rms_g2o_error_ground5.push_back(dist5);
//     rms_g2o_error_ground.push_back(rms_dist);
//   return rms_dist;
// }




double ground_plane_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double rms_dist = 0;
  double dist = 0;

  extract_frame_cloud(frame_cloud, plane_points_ground1, plane_idx_ground1, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_ground1.max_index[vertexID - 1] - plane_idx_ground1.min_index[vertexID - 1] + 1); i++)
    dist += pow(frame_pts_distances[i], 2);
  rms_dist = dist;

  rms_g2o_error_ground1.push_back(rms_dist);


rms_g2o_error_ground.push_back(rms_dist);
  return rms_dist;
}



double ground_plane2_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist = 0;
  extract_frame_cloud(frame_cloud, plane_points_ground2, plane_idx_ground2, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_ground2.max_index[vertexID - 1] - plane_idx_ground2.min_index[vertexID - 1] + 1); i++)
  {
    dist += pow(frame_pts_distances[i], 2);
  }
  rms_dist = dist;
  rms_g2o_error_ground2.push_back(rms_dist);

  // std::cout<<"F_ID:"<<vertexID<<" rms_dist diff ground plane2: "<< rms_dist <<"  " <<dist << "\n";
  // std::cout<<"F_ID:"<<vertexID<<" rms_dist diff ground plane:"<<rms_dist-rms_distances[vertexID-1]<<"\n";
  return rms_dist;
}

double ground_plane3_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist = 0;
  extract_frame_cloud(frame_cloud, plane_points_ground3, plane_idx_ground3, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_ground3.max_index[vertexID - 1] - plane_idx_ground3.min_index[vertexID - 1] + 1); i++)
  {
    dist += pow(frame_pts_distances[i], 2);
  }
  rms_dist = dist;
  rms_g2o_error_ground3.push_back(rms_dist);

  // std::cout<<"F_ID:"<<vertexID<<" rms_dist diff ground plane2: "<< rms_dist <<"  " <<dist << "\n";
  // std::cout<<"F_ID:"<<vertexID<<" rms_dist diff ground plane:"<<rms_dist-rms_distances[vertexID-1]<<"\n";
  return rms_dist;
}


double ground_plane4_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist = 0;
  extract_frame_cloud(frame_cloud, plane_points_ground4, plane_idx_ground4, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_ground4.max_index[vertexID - 1] - plane_idx_ground4.min_index[vertexID - 1] + 1); i++)
  {
    dist += pow(frame_pts_distances[i], 2);
  }
  rms_dist = dist;
  rms_g2o_error_ground4.push_back(rms_dist);

  // std::cout<<"F_ID:"<<vertexID<<" rms_dist diff ground plane2: "<< rms_dist <<"  " <<dist << "\n";
  // std::cout<<"F_ID:"<<vertexID<<" rms_dist diff ground plane:"<<rms_dist-rms_distances[vertexID-1]<<"\n";
  return rms_dist;
}

//compute the frame wise rms error for wall plane1 in order to calculate the error vector
double wall_plane1_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall = 0;
  extract_frame_cloud(frame_cloud, plane_points_wall1, plane_idx_wall1, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_wall1, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_wall1.max_index[vertexID - 1] - plane_idx_wall1.min_index[vertexID - 1] + 1); i++)
    dist += pow(frame_pts_distances[i], 2);
  rms_dist_wall = dist;

  rms_g2o_error_wall1.push_back(rms_dist_wall);

  // std::cout<<"F_ID Wall1:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall1[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//compute the frame wise rms error for wall plane2 in order to calculate the error vector
double wall_plane2_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall = 0;
  extract_frame_cloud(frame_cloud, plane_points_wall2, plane_idx_wall2, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_wall2, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_wall2.max_index[vertexID - 1] - plane_idx_wall2.min_index[vertexID - 1] + 1); i++)
    dist += pow(frame_pts_distances[i], 2);
  rms_dist_wall = dist;
  rms_g2o_error_wall2.push_back(rms_dist_wall);

  //std::cout<<"F_ID Wall2:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall2[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//compute the frame wise rms error for wall plane3 in order to calculate the error vector
double wall_plane3_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall = 0;
  extract_frame_cloud(frame_cloud, plane_points_wall3, plane_idx_wall3, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_wall3, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_wall3.max_index[vertexID - 1] - plane_idx_wall3.min_index[vertexID - 1] + 1); i++)
    dist += pow(frame_pts_distances[i], 2);
  rms_dist_wall = dist;
  rms_g2o_error_wall3.push_back(rms_dist_wall);

  //std::cout<<"F_ID Wall3:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall3[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//compute the frame wise rms error for wall plane4 in order to calculate the error vector
double wall_plane4_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall = 0;
  extract_frame_cloud(frame_cloud, plane_points_wall4, plane_idx_wall4, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_wall4, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_wall4.max_index[vertexID - 1] - plane_idx_wall4.min_index[vertexID - 1] + 1); i++)
    dist += pow(frame_pts_distances[i], 2);
  rms_dist_wall = dist;
  rms_g2o_error_wall4.push_back(rms_dist_wall);

  //std::cout<<"F_ID Wall4:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall4[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//compute the frame wise rms error for wall plane5 in order to calculate the error vector
double wall_plane5_rms_error_vertexi(Eigen::Isometry3d transform_mat, int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rms_dist_wall = 0;
  extract_frame_cloud(frame_cloud, plane_points_wall5, plane_idx_wall5, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
  compute_distance(transformed_cloud, coeffs_wall5, frame_pts_distances);
  //compute the rms distance
  for (size_t i = 0; i < (plane_idx_wall5.max_index[vertexID - 1] - plane_idx_wall5.min_index[vertexID - 1] + 1); i++)
    dist += pow(frame_pts_distances[i], 2);
  rms_dist_wall = dist;
  rms_g2o_error_wall5.push_back(rms_dist_wall);

  // std::cout<<"F_ID Wall5:"<<vertexID<<" rms_dist diff wall plane:"<<rms_dist_wall-rms_distances_wall5[vertexID-1]<<"\n";
  return rms_dist_wall;
}

//extract the frame points cloud from the fitted plane cloud
void extract_frame_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr plane_cloud, key_index plane_index, int vertexID)
{
  frame_cloud->width = plane_index.max_index[vertexID - 1] - plane_index.min_index[vertexID - 1] + 1;
  frame_cloud->height = 1;
  frame_cloud->is_dense = false;
  frame_cloud->points.resize(frame_cloud->width * frame_cloud->height);

  for (size_t i = 0; i < frame_cloud->points.size(); ++i)
  {
    frame_cloud->points[i] = plane_cloud->points[plane_index.min_index[vertexID - 1] + i];
  }
}

//transform the points cloud according to the transformation matrix
void trnasform_point_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, pcl::PointCloud<PointT>::Ptr transformed_cloud, Eigen::Isometry3d transform_mat)
{
  Eigen::Isometry3f trans_mat;
  trans_mat = transform_mat.cast<float>();
  //std::cout<<"Trans_mat:"<<trans_mat.matrix()<<"\n";
  pcl::transformPointCloud(*frame_cloud, *transformed_cloud, trans_mat);
}

//load the text file for wall information
void load_wall_info(std::string file, std::vector<int> &roi_wall_idx, std::vector<int> &inlier_wall_idx, Eigen::VectorXf &coeffs_wall)
{
  ifstream wfile;
  std::string line;
  int size, data;
  wfile.open(file);
  if (!wfile.is_open())
  {
    std::cerr << "unable to open wall_seg file\n";
    return;
  }
  std::getline(wfile, line);
  //std::cout<<line<<std::endl;
  std::getline(wfile, line);
  //std::cout<<line<<std::endl;
  wfile >> size;
  //std::cout<<size<<std::endl;
  std::getline(wfile, line);
  std::getline(wfile, line);
  //std::cout<<line<<std::endl;
  for (size_t i = 0; i < size; i++)
  {
    wfile >> data;
    roi_wall_idx.push_back(data);
  }
  //for(size_t i=0; i<size; i++)
  //std::cout<<" "<<roi_wall_idx[i];
  std::getline(wfile, line);
  std::getline(wfile, line);
  //std::cout<<line<<std::endl;
  wfile >> coeffs_wall(0) >> coeffs_wall(1) >> coeffs_wall(2) >> coeffs_wall(3);
  //std::cout<<coeffs_wall<<std::endl;
  std::getline(wfile, line);
  std::getline(wfile, line);
  //std::cout<<line<<std::endl;
  std::getline(wfile, line);
  //std::cout<<line<<std::endl;
  wfile >> size;
  //std::cout<<size<<std::endl;
  std::getline(wfile, line);
  std::getline(wfile, line);
  //std::cout<<line<<std::endl;
  for (size_t i = 0; i < size; i++)
  {
    wfile >> data;
    inlier_wall_idx.push_back(data);
  }
  //for(size_t i=0; i<size; i++)
  //std::cout<<" "<<inlier_wall_idx[i];
  wfile.close();
}

//========================================================
// Modified by Mahmood
//Calculate the mean and standard deviation of distances
void calculate_mean_sd_distances(std::vector<double> dist, double &mean, double &sd)
{
  double sum = 0.0;
  int N = 0;
  for (auto x : dist)
  {
    sum += x;
    N++;
  }
  mean = sum / N;
  sum = 0.0;
  for (auto x : dist)
  {
    sum += pow(x - mean, 2);
  }
  sd = sqrt(sum / N);
}

//============================================

//Calculate the Global RSS distance for plane points to the plane
double calc_globalRSS_after_optimization(pcl::PointCloud<PointT>::Ptr plane_pts, key_index plane_index, Eigen::VectorXf coeff, std::vector<double> &distances, std::vector<double> &frame_wise_rss)
{
  double global_rss = 0.0;
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  g2o::VertexSE3 *node;
  std::vector<double> frame_pts_distances;
  // std::cout<<"  node_array.size();: "<<  node_array.size() <<std::endl;
  for (int i = 0; i < node_array.size(); i++)
  {
    if ((plane_index.min_index[i] == -1) && (plane_index.max_index[i] == -1))
    {
      double dist = 0.0;
      frame_wise_rss.push_back(dist);
      continue;
    }
    else
    {
      extract_frame_cloud(frame_cloud, plane_pts, plane_index, i + 1);
      node = node_array[i];
      Eigen::Isometry3d transform_mat = node->estimate();
      trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
      compute_distance(transformed_cloud, coeff, frame_pts_distances);
      //compute the rss distance
      double dist = 0.0;
      for (size_t j = 0; j < (plane_index.max_index[i] - plane_index.min_index[i] + 1); j++)
      {
        dist += pow(frame_pts_distances[j], 2);
        distances.push_back(frame_pts_distances[j]);
      }

      global_rss += dist;
      frame_wise_rss.push_back(dist);
    }
  }
  // std::cout<<"  frame_wise_rss.size();: "<<  frame_wise_rss.size() <<std::endl;

  return global_rss;
}

//============================================
void display_RSSE()
{

  double mean, sd;

  double global_rss_ground1 = 0.0;
  double global_rss_ground2 = 0.0;
  double global_rss_ground3 = 0.0;
  double global_rss_ground4 = 0.0;
  double global_rss_ground5 = 0.0;
  double global_rss_wall1 = 0.0;
  double global_rss_wall2 = 0.0;
  double global_rss_wall3 = 0.0;
  double global_rss_wall4 = 0.0;
  double global_rss_wall5 = 0.0;
  double global_rss_all_wall_after = 0.0;

  double global_rss_ground1_before = 0.0;
  double global_rss_ground2_before = 0.0;
  double global_rss_ground3_before = 0.0;
  double global_rss_ground4_before = 0.0;
  double global_rss_ground5_before = 0.0;
  double global_rss_wall1_before = 0.0;
  double global_rss_wall2_before = 0.0;
  double global_rss_wall3_before = 0.0;
  double global_rss_wall4_before = 0.0;
  double global_rss_wall5_before = 0.0;
  double global_rss_all_wall_before = 0.0;

  std::cout << "==============================" << std::endl;
  std::cout << " display_RSSE() is started " << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << "number_of_ground_planes " <<number_of_ground_planes<< std::endl;
  std::cout << "number_of_wall_planes " <<number_of_wall_planes<< std::endl;

  global_rss_ground1_before = 0.0;
  global_rss_ground1 = 0.0;
  for (double rss : rms_distances_ground1)
  {
    global_rss_ground1_before += rss;
  }
  std::cout << "The global RSS for ground plane1 before optimization is: " << global_rss_ground1_before << std::endl;
  global_rss_ground1 = calc_globalRSS_after_optimization(plane_points_ground1, plane_idx_ground1, coeffs_ground, after_opti_distances_ground1, after_opti_rms_distances_ground1);
  std::cout << "The global RSS for ground plane1 after optimization is: " << global_rss_ground1 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_ground1, mean, sd);
  std::cout << "Mean and SD distances for ground plane1 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;

  
   // ground 2
     global_rss_ground2_before = 0.0;
  global_rss_ground2 = 0.0;
    if (number_of_ground_planes >= 2) {
  for (double rss : rms_distances_ground2)
  {
    global_rss_ground2_before += rss;
  }
  std::cout << "The global RSS for ground plane2 before optimization is: " << global_rss_ground2_before << std::endl;
  global_rss_ground2 = calc_globalRSS_after_optimization(plane_points_ground2, plane_idx_ground2, coeffs_ground, after_opti_distances_ground2, after_opti_rms_distances_ground2);
  std::cout << "The global RSS for ground plane2 after optimization is: " << global_rss_ground2 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_ground2, mean, sd);
  std::cout << "Mean and SD distances for ground plane2 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;
    }

    // ground 3
      global_rss_ground3_before = 0.0;
  global_rss_ground3 = 0.0;
      if (number_of_ground_planes >= 3) {
  for (double rss : rms_distances_ground3)
  {
    global_rss_ground3_before += rss;
  }
  std::cout << "The global RSS for ground plane3 before optimization is: " << global_rss_ground3_before << std::endl;
  global_rss_ground3 = calc_globalRSS_after_optimization(plane_points_ground3, plane_idx_ground3, coeffs_ground, after_opti_distances_ground3, after_opti_rms_distances_ground3);
  std::cout << "The global RSS for ground plane3 after optimization is: " << global_rss_ground3 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_ground3, mean, sd);
  std::cout << "Mean and SD distances for ground plane3 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;
      }

// ground 4
  global_rss_ground4_before = 0.0;
  global_rss_ground4 = 0.0;
  if (number_of_ground_planes >= 4) {
  for (double rss : rms_distances_ground4)
  {
    global_rss_ground4_before += rss;
  }
  std::cout << "The global RSS for ground plane4 before optimization is: " << global_rss_ground4_before << std::endl;
  global_rss_ground4 = calc_globalRSS_after_optimization(plane_points_ground4, plane_idx_ground4, coeffs_ground, after_opti_distances_ground4, after_opti_rms_distances_ground4);
  std::cout << "The global RSS for ground plane4 after optimization is: " << global_rss_ground4 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_ground4, mean, sd);
  std::cout << "Mean and SD distances for ground plane4 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;
  }


// ground 5
  global_rss_ground5_before = 0.0;
  global_rss_ground5 = 0.0;
  if (number_of_ground_planes >= 5) {
  for (double rss : rms_distances_ground5)
  {
    global_rss_ground5_before += rss;
  }
  std::cout << "The global RSS for ground plane5 before optimization is: " << global_rss_ground5_before << std::endl;
  global_rss_ground5 = calc_globalRSS_after_optimization(plane_points_ground5, plane_idx_ground5, coeffs_ground, after_opti_distances_ground5, after_opti_rms_distances_ground5);
  std::cout << "The global RSS for ground plane5 after optimization is: " << global_rss_ground5 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_ground5, mean, sd);
  std::cout << "Mean and SD distances for ground plane5 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;
  }

  // wall 1
  global_rss_wall1_before = 0.0;
  global_rss_wall1 = 0.0;
  if (number_of_wall_planes >= 1) {
  for (double rss_wall1 : rms_distances_wall1)
  {
    global_rss_wall1_before += rss_wall1;
  }
  std::cout << "The global RSS for wall plane1 before optimization is: " << global_rss_wall1_before << std::endl;
  global_rss_wall1 = calc_globalRSS_after_optimization(plane_points_wall1, plane_idx_wall1, coeffs_wall1, after_opti_distances_wall1, after_opti_rms_distances_wall1);
  std::cout << "The global RSS for wall plane1 after optimization is: " << global_rss_wall1 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_wall1, mean, sd);
  std::cout << "Mean and SD distances for wall plane-1 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;
  }
  // wall 2
  global_rss_wall2 = 0.0;
  global_rss_wall2_before = 0.0;
  if (number_of_wall_planes >= 2) {
  for (double rss_wall2 : rms_distances_wall2)
  {
    global_rss_wall2_before += rss_wall2;
  }
  std::cout << "The global RSS for wall plane2 before optimization is: " << global_rss_wall2_before << std::endl;
  global_rss_wall2 = calc_globalRSS_after_optimization(plane_points_wall2, plane_idx_wall2, coeffs_wall2, after_opti_distances_wall2, after_opti_rms_distances_wall2);
  std::cout << "The global RSS for wall plane2 after optimization is: " << global_rss_wall2 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_wall2, mean, sd);
  std::cout << "Mean and SD distances for wall plane2 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;
  }
  // wall 3
    global_rss_wall3_before = 0.0;
  global_rss_wall3 = 0.0;
  if (number_of_wall_planes >= 3) {
  for (double rss_wall3 : rms_distances_wall3)
  {
    global_rss_wall3_before += rss_wall3;
  }
  std::cout << "The global RSS for wall plane3 before optimization is: " << global_rss_wall3_before << std::endl;
  global_rss_wall3 = calc_globalRSS_after_optimization(plane_points_wall3, plane_idx_wall3, coeffs_wall3, after_opti_distances_wall3, after_opti_rms_distances_wall3);
  std::cout << "The global RSS for wall plane3 after optimization is: " << global_rss_wall3 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_wall3, mean, sd);
  std::cout << "Mean and SD distances for wall plane3 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;
  }
  // wall 4
    global_rss_wall4_before = 0.0;
  global_rss_wall4 = 0.0;
  if (number_of_wall_planes >= 4) {
  for (double rss_wall4 : rms_distances_wall4)
  {
    global_rss_wall4_before += rss_wall4;
  }
  std::cout << "The global RSS for wall plane4 before optimization is: " << global_rss_wall4_before << std::endl;
  global_rss_wall4 = calc_globalRSS_after_optimization(plane_points_wall4, plane_idx_wall4, coeffs_wall4, after_opti_distances_wall4, after_opti_rms_distances_wall4);
  std::cout << "The global RSS for wall plane4 after optimization is: " << global_rss_wall4 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_wall4, mean, sd);
  std::cout << "Mean and SD distances for wall plane4 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  std::cout << " " << std::endl;
  }
  // wall 5
    global_rss_wall5_before = 0.0;
  global_rss_wall5 = 0.0;
  if (number_of_wall_planes >= 5) {
  for (double rss_wall5 : rms_distances_wall5)
  {
    global_rss_wall5_before += rss_wall5;
  }
  std::cout << "The global RSS for wall plane5 before optimization is: " << global_rss_wall5_before << std::endl;
  global_rss_wall5 = calc_globalRSS_after_optimization(plane_points_wall5, plane_idx_wall5, coeffs_wall5, after_opti_distances_wall5, after_opti_rms_distances_wall5);
  std::cout << "The global RSS for wall plane5 after optimization is: " << global_rss_wall5 << std::endl;
  calculate_mean_sd_distances(after_opti_distances_wall5, mean, sd);
  std::cout << "Mean and SD distances for wall plane5 after optimization are: " << mean << "\t" << sd << std::endl;
  std::cout << "==============================" << std::endl;
  }
  
  // all plane error
  std::cout << " " << std::endl;
  std::cout << "==============================" << std::endl;

  std::cout << "The global RSS for all planes" << std::endl;

  global_rss_all_wall_before = global_rss_wall1_before + global_rss_wall2_before + global_rss_wall3_before + global_rss_wall4_before + global_rss_wall5_before + global_rss_ground1_before + global_rss_ground2_before;
  global_rss_all_wall_after = global_rss_wall1 + global_rss_wall2 + global_rss_wall3 + global_rss_wall4 + global_rss_wall5 + global_rss_ground1 + global_rss_ground2;
  std::cout << "The global RSS for all planes before optimization is: " << global_rss_all_wall_before << std::endl;
  std::cout << "The global RSS for all planes After optimization is: " << global_rss_all_wall_after << std::endl;
  std::cout << "percent change The global RSS for all planes After optimization is: " << 100 * (global_rss_all_wall_before - global_rss_all_wall_after) / global_rss_all_wall_before << std::endl;
  std::cout << "==============================" << std::endl;
}
