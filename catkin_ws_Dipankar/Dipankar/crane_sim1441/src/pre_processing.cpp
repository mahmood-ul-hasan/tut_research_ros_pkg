#include<pre_processing.hpp>
#include<key_index.hpp>
#include<graph_optimization.hpp>
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
void fit_plane(pcl::PointCloud<PointT>::Ptr cloud_filterred, std::vector<float> coeffs_set[3], std::vector<int> inliers_set[3], std::vector<int> outliers_set[3], int num_plane, double dist_thres)
{
  pcl::PointCloud<PointT>::Ptr cloud_original (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
  std::string coeffs_file = "plane_coefficient.txt";
    
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());

  std::vector<double>plane_dist;
  Eigen::VectorXf coeffs_wall;
  pcl::PointCloud<PointT>::Ptr origin_point (new pcl::PointCloud<PointT>);
  create_origin_point(origin_point);
      
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  //seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(dist_thres);   

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
   
  seg.setEpsAngle(pcl::deg2rad(40.0));
    
  seg.setMaxIterations (1000);
     
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract(true);
    
  int i = 0;
     
  //Extract 4 planes
  while (i < num_plane)
  {
    // Segment the largest planar component from the remaining cloud
    if(i==0)                    //ground plane; plane perpendicular to the z-axis
      axis << 0.0, 0.0, 1.0;
    else if(i==1)               //plane perpendicular to the x-axis
      axis << 1.0, 0.0, 0.0;
    else if(i==2)                       //plane perpendicular to the y-axis
      axis << 0.0, 1.0, 0.0;
    else if(i==3)
      axis << 0.0, 1.0, 0.0;

    seg.setAxis(axis);
    seg.setInputCloud (cloud_filterred);
    seg.segment (*inliers, *coeff);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
   	}
    inliers_set[i] = inliers->indices;
    coeffs_set[i] = coeff->values;
    // Extract the inliers
    extract.setInputCloud (cloud_filterred);
    extract.setIndices (inliers);
   	extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
      
    //set the plane points for the detected planes, first ground plane, second plane for wall1 and third plane for wall2
    if(i == 0)   //ground plane
      *plane_points_ground = *cloud_p;
    else if(i == 1)   //wall plane1
      *plane_points_wall1 = *cloud_p;
    else if(i == 2)   //wall plane2
      *plane_points_wall2 = *cloud_p;
    else if(i == 3)   //wall plane2
      *plane_points_wall3 = *cloud_p;

    // save the detected planes
    std::stringstream ss;
    ss << "new_pcl" << i << ".pcd";
	  pcl::io::savePCDFileASCII(ss.str (), *cloud_p);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter(outliers_set[i]);
    extract.setNegative (true);
    extract.setKeepOrganized (false);
    extract.filter (*cloud_f);
    cloud_filterred.swap (cloud_f);
    i++;
    
    if( i > 1)
    {
      coeffs_wall = Eigen::Vector4f(coeff->values[0],coeff->values[1],coeff->values[2],coeff->values[3]);
      compute_distance(origin_point, coeffs_wall, plane_dist);
      std::cout<<"Dist: "<<plane_dist[0]<<std::endl;
      if(plane_dist[0] <= 20)
        i--;
    }
  }
  save_plane_coefficients(coeffs_set, num_plane, coeffs_file);
}

//compute the distance of the inliers data to the plane
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeff, std::vector<double> &distances)
{
  //Compute the distances to the model
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_inliers(new pcl::SampleConsensusModelPlane<PointT> (final));
  model_inliers->getDistancesToModel(coeff, distances);
}

//compute the frame_id wise average rss distance to the plane
void compute_rss_distance(std::vector<double> &rss_distances, std::vector<double> &distances, key_index inliers_idx)
{
  for(int i:inliers_idx.frame_id){
      double dist = 0;
      if((inliers_idx.min_index[i] == -1) && (inliers_idx.max_index[i] == -1))
        dist = 0;
      else{
        for(size_t j= inliers_idx.min_index[i]; j <= inliers_idx.max_index[i]; j++)
          dist += pow(distances[j],2);
      }
      rss_distances.push_back(sqrt(dist));
  }
}

//compute the frame wise rss error for ground plane in order to calculate the error vector
double ground_plane_rss_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rss_dist;
  extract_frame_cloud(frame_cloud, plane_points_ground, plane_idx_ground, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_ground, frame_pts_distances);
  //compute the rss distance
  for(size_t i = 0; i<(plane_idx_ground.max_index[vertexID-1] - plane_idx_ground.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rss_dist = sqrt(dist);
  
  //std::cout<<"F_ID:"<<vertexID<<" rss_dist diff ground plane:"<<rss_dist-rss_distances_ground[vertexID-1]<<"\n";
  return rss_dist;
}


//compute the frame wise rss error for wall plane1 in order to calculate the error vector
double wall_plane1_rss_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rss_dist_wall;
  extract_frame_cloud(frame_cloud, plane_points_wall1, plane_idx_wall1, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_wall1, frame_pts_distances);
  //compute the rss distance
  for(size_t i = 0; i<(plane_idx_wall1.max_index[vertexID-1] - plane_idx_wall1.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rss_dist_wall = sqrt(dist);
  
  //std::cout<<"F_ID Wall1:"<<vertexID<<" rss_dist diff wall plane:"<<rss_dist_wall-rss_distances_wall1[vertexID-1]<<"\n";
  return rss_dist_wall;
}

//compute the frame wise rss error for wall plane2 in order to calculate the error vector
double wall_plane2_rss_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rss_dist_wall;
  extract_frame_cloud(frame_cloud, plane_points_wall2, plane_idx_wall2, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_wall2, frame_pts_distances);
  //compute the rss distance
  for(size_t i = 0; i<(plane_idx_wall2.max_index[vertexID-1] - plane_idx_wall2.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rss_dist_wall = sqrt(dist);
  
  //std::cout<<"F_ID Wall2:"<<vertexID<<" rss_dist diff wall plane:"<<rss_dist_wall-rss_distances_wall2[vertexID-1]<<"\n";
  return rss_dist_wall;
}

//compute the frame wise rss error for wall plane2 in order to calculate the error vector
double wall_plane3_rss_error_vertexi(Eigen::Isometry3d transform_mat,int vertexID)
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  std::vector<double> frame_pts_distances;
  double dist = 0, rss_dist_wall;
  extract_frame_cloud(frame_cloud, plane_points_wall3, plane_idx_wall3, vertexID);
  trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
  compute_distance(transformed_cloud, coeffs_wall3, frame_pts_distances);
  //compute the rss distance
  for(size_t i = 0; i<(plane_idx_wall3.max_index[vertexID-1] - plane_idx_wall3.min_index[vertexID-1] + 1); i++)
    dist+= pow(frame_pts_distances[i],2);
  rss_dist_wall = sqrt(dist);
  
  //std::cout<<"F_ID Wall3:"<<vertexID<<" rss_dist diff wall plane:"<<rss_dist_wall-rss_distances_wall3[vertexID-1]<<"\n";
  return rss_dist_wall;
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

//Calculate the Global RSS Distances before and after optimization
void display_RSSE()
{  
  std::vector<double> distances_ground;
  std::vector<double> distances_wall1;
  std::vector<double> distances_wall2;
  std::vector<double> distances_wall3;
  double mean, sd;
  double global_rss = 0.0;
  for(double rss:rss_distances_ground){
    global_rss += rss;
  }
  std::cout<<"The global RSS for ground plane before optimization is: "<<global_rss<<std::endl;
  global_rss = calc_globalRSS_after_optimization(plane_points_ground, plane_idx_ground, coeffs_ground, distances_ground);
  std::cout<<"The global RSS for ground plane after optimization is: "<<global_rss<<std::endl;
  calculate_mean_sd_distances(distances_ground, mean, sd);
  std::cout<<"Mean and SD distances for ground plane after optimization are: "<<mean<<"\t"<<sd<<std::endl;

  if(W1){
    global_rss = 0.0;
    for(double rss:rss_distances_wall1){
      global_rss += rss;
    }
    std::cout<<"The global RSS for wall plane1 before optimization is: "<<global_rss<<std::endl;
    global_rss = calc_globalRSS_after_optimization(plane_points_wall1, plane_idx_wall1, coeffs_wall1, distances_wall1);
    std::cout<<"The global RSS for wall plane1 after optimization is: "<<global_rss<<std::endl;
    calculate_mean_sd_distances(distances_wall1, mean, sd);
    std::cout<<"Mean and SD distances for wall plane-1 after optimization are: "<<mean<<"\t"<<sd<<std::endl;
  }

  if(W2){
    global_rss = 0.0;
    for(double rss:rss_distances_wall2){
      global_rss += rss;
    }
    std::cout<<"The global RSS for wall plane2 before optimization is: "<<global_rss<<std::endl;
    global_rss = calc_globalRSS_after_optimization(plane_points_wall2, plane_idx_wall2, coeffs_wall2, distances_wall2);
    std::cout<<"The global RSS for wall plane2 after optimization is: "<<global_rss<<std::endl;
    calculate_mean_sd_distances(distances_wall2, mean, sd);
    std::cout<<"Mean and SD distances for wall plane-2 after optimization are: "<<mean<<"\t"<<sd<<std::endl;
  }

  if(W3){
    global_rss = 0.0;
    for(double rss:rss_distances_wall3){
      global_rss += rss;
    }
    std::cout<<"The global RSS for wall plane3 before optimization is: "<<global_rss<<std::endl;
    global_rss = calc_globalRSS_after_optimization(plane_points_wall3, plane_idx_wall3, coeffs_wall3, distances_wall3);
    std::cout<<"The global RSS for wall plane3 after optimization is: "<<global_rss<<std::endl;
    calculate_mean_sd_distances(distances_wall3, mean, sd);
    std::cout<<"Mean and SD distances for wall plane-3 after optimization are: "<<mean<<"\t"<<sd<<std::endl;
  }
}

//Calculate the mean and standard deviation of distances
void calculate_mean_sd_distances(std::vector<double> dist, double &mean, double &sd)
{
  double sum = 0.0;
  int N = 0;
  for(auto x:dist){
    sum += x;
    N++;
  }
  mean = sum / N;
  sum = 0.0;
  for(auto x:dist){
    sum += pow(x - mean, 2);
  }
  sd = sqrt(sum / N);
}

//Save original index in a file for later use in successive iteration
void save_filterred_index(key_index idx, std::string idx_file)
{
  ofstream wfile;
  wfile.open(idx_file);
  if(!wfile.is_open()){
    std::cerr<<"unable to open filterred_index file\n";
    return;
  }
  wfile<<"Frame Size"<<"\n";
  wfile<<idx.frame_id.size()<<"\n";
  wfile<<"Frame_id"<<"\t"<<"Min_index"<<"\t"<<"Max_index"<<"\n";
  for(int i:idx.frame_id)
    wfile<<idx.frame_id[i]<<"\t\t"<<idx.min_index[i]<<"\t\t"<<idx.max_index[i]<<"\n";

  wfile.close();
}

//Load original index from the original_index.txt file
void load_filterred_index(key_index &filterred_index, std::string idx_file)
{
  ifstream wfile;
  std::string line;
  int size,data;
  wfile.open(idx_file);
  if(!wfile.is_open()){
    std::cerr<<"unable to open filter_idx file\n";
    return;
  }
  std::getline(wfile,line);
  wfile>>size;
  std::getline(wfile,line);
  std::getline(wfile,line);
  for(int i=0; i<size;i++){
    wfile>>data;
    filterred_index.frame_id.push_back(data);
    wfile>>data;
    filterred_index.min_index.push_back(data);
    wfile>>data;
    filterred_index.max_index.push_back(data);
  }

  wfile.close();
}

//Create the convex hull and compute the area of the convex hull to calculate the area of the plane
double compute_plane_area(pcl::PointCloud<PointT>::Ptr input_cloud)
{
  double area, volume;
  pcl::PointCloud<PointT>::Ptr cloud_hull(new pcl::PointCloud<PointT>);
  pcl::ConvexHull<PointT> cHull;
  cHull.setInputCloud (input_cloud);
  cHull.setComputeAreaVolume(true);
  cHull.reconstruct (*cloud_hull);
  area = cHull.getTotalArea();
  volume = cHull.getTotalVolume();
  std::cout<<"Hull Area: "<<area<<std::endl;
  std::cout<<"Hull Volume: "<<volume<<std::endl;
  return area;
}

//Create the origin point
void create_origin_point(pcl::PointCloud<PointT>::Ptr origin_point)
{
  origin_point->width    = 1;
  origin_point->height   = 1;
  origin_point->is_dense = false;
  origin_point->points.resize (origin_point->width * origin_point->height);
  origin_point->points[0].x = 0.0;
  origin_point->points[0].y = 0.0;
  origin_point->points[0].z = 0.0;
  origin_point->points[0].intensity = 0.0;
}

//Save plane coefficients into a file
void save_plane_coefficients(std::vector<float> coeffs_set[3], int num_plane, std::string coeffs_file)
{
  ofstream wfile;
  wfile.open(coeffs_file);
  if(!wfile.is_open()){
    std::cerr<<"unable to open coefficient file\n";
    return;
  }
  for(int i = 0; i < num_plane; i++){
    wfile<<"Coefficient for Plane_"<<i<<" is:""\n";
    wfile<<coeffs_set[i][0]<<" "<<coeffs_set[i][1]<<" "<<coeffs_set[i][2]<<" "<<coeffs_set[i][3]<<"\n";
  }

  wfile.close();
}
