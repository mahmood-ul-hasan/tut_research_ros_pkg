#include<graph_optimization.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include<pre_processing.hpp>
#include<key_index.hpp>

#include <iostream>
#include <cmath>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_ground_plane.hpp>
#include <g2o/edge_se3_wall_plane1.hpp>
#include <g2o/edge_se3_wall_plane2.hpp>

#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;

std::unique_ptr<GraphSLAM> graph_slam;
std::vector<g2o::VertexSE3*> node_array;

void graph_construction()
{
  g2o::VertexSE3* prev_node, *cur_node; 
  g2o::VertexPlane* ground_plane_node;
  g2o::VertexPlane* wall_plane_node1;
  g2o::VertexPlane* wall_plane_node2;
  g2o::VertexPlane* wall_plane_node3;
  
  std::unique_ptr<InformationMatrixCalculator> info_calculator;

  double floor_edge_stddev = 10;
  double wall_edge_stddev1 = 20;
  double wall_edge_stddev2 = 20;
  double wall_edge_stddev3 = 20;
    
  /*double ground_edge_robust_kernel_size = 3.0;
  std::string ground_edge_robust_kernel ="Huber";
  double node_edge_robust_kernel_size = 3.0;
  std::string node_edge_robust_kernel ="Huber";*/
  
  ground_plane_node = nullptr;
  wall_plane_node1 = nullptr;
  wall_plane_node2 = nullptr;
  wall_plane_node3 = nullptr;
  
  //Initialize
  graph_slam.reset(new GraphSLAM("lm_var"));
  info_calculator.reset(new InformationMatrixCalculator());

  //Add floor plane node
  ground_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_ground[0], coeffs_ground[1], coeffs_ground[2], coeffs_ground[3]));
  ground_plane_node->setFixed(true);
  
  Eigen::Isometry3d sen_pose = Eigen::Isometry3d::Identity();
  prev_node = graph_slam->add_se3_node(sen_pose);
  node_array.push_back(prev_node);
  
  //Create and set relative pose to 0
  Eigen::Isometry3d rel_pose;
  Eigen::Matrix4d x = Eigen::Matrix4d::Identity();
  rel_pose.matrix() = x;
  
  //Add an edge between floor plan node and first pose node
  //Eigen::Vector4d coeffs(floor_coeffs[0], floor_coeffs[1], floor_coeffs[2], floor_coeffs[3]);
  Eigen::Vector4d measurement_ground(0,0,0,0);
  Eigen::Matrix3d information_ground = Eigen::Matrix3d::Identity();
  information_ground(0,0) = 1.0 / floor_edge_stddev;
  //information_ground(1,1) = 0;//1.0 / floor_edge_stddev;
  //information_ground(2,2) = 0;//1.0 / floor_edge_stddev;
  
  auto edge = graph_slam->add_se3_ground_plane_edge(prev_node, ground_plane_node, measurement_ground, information_ground);
  //graph_slam->add_robust_kernel(edge, ground_edge_robust_kernel, ground_edge_robust_kernel_size);
  
  Eigen::MatrixXd info_mat;
  //info_mat = info_calculator->calc_const_information_matrix();
  //std::cout<<"Edge Info. Matrix: "<<info_mat<<std::endl;
  
  //Add the successive nodes and edges to the graph using the sensor pose and constant information matrix
  for(int i = 1; i < plane_idx_ground.frame_id.size(); i++)
  {
    cur_node = graph_slam->add_se3_node(sen_pose);
    node_array.push_back(cur_node);
    info_mat = info_calculator->calc_const_information_matrix();
    auto edge1 = graph_slam->add_se3_edge(prev_node, cur_node, rel_pose, info_mat);
    //graph_slam->add_robust_kernel(edge1, node_edge_robust_kernel, node_edge_robust_kernel_size);
    if((plane_idx_ground.min_index[i] != -1) && (plane_idx_ground.max_index[i] != -1)){
      auto edge2 = graph_slam->add_se3_ground_plane_edge(cur_node, ground_plane_node, measurement_ground, information_ground);
      //graph_slam->add_robust_kernel(edge2, ground_edge_robust_kernel, ground_edge_robust_kernel_size);
    }
    prev_node = cur_node;
  }
 
  //add wall_plane_nodes and wall_plane_edges to the graph
  if(W1){
    wall_plane_node1 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall1[0], coeffs_wall1[1], coeffs_wall1[2], coeffs_wall1[3]));
    wall_plane_node1->setFixed(true);
  }
  if(W2){
    wall_plane_node2 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall2[0], coeffs_wall2[1], coeffs_wall2[2], coeffs_wall2[3]));
    wall_plane_node2->setFixed(true);
  }
  if(W3){
    wall_plane_node3 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall3[0], coeffs_wall3[1], coeffs_wall3[2], coeffs_wall3[3]));
    wall_plane_node3->setFixed(true);
  }
  
  Eigen::Vector4d measurement_wall(0,0,0,0);
  Eigen::Matrix3d information_wall1 = Eigen::Matrix3d::Identity();
  information_wall1(0,0) = 1.0 / wall_edge_stddev1;
  //information_wall1(1,1) = 0;//1.0 / wall_edge_stddev1;
  //information_wall1(2,2) = 0;//1.0 / wall_edge_stddev1;
  
  Eigen::Matrix3d information_wall2 = Eigen::Matrix3d::Identity();
  information_wall2(0,0) = 1.0 / wall_edge_stddev2;
  //information_wall2(1,1) = 0;//1.0 / wall_edge_stddev2;
  //information_wall2(2,2) = 0;//1.0 / wall_edge_stddev2;

  Eigen::Matrix3d information_wall3 = Eigen::Matrix3d::Identity();
  information_wall3(0,0) = 1.0 / wall_edge_stddev3;
  //information_wall3(1,1) = 0;//1.0 / wall_edge_stddev3;
  //information_wall3(2,2) = 0;//1.0 / wall_edge_stddev3;

  for(int i = 0; i < plane_idx_wall1.frame_id.size(); i++)
  {
     if(W1){
       if((plane_idx_wall1.min_index[i] != -1) && (plane_idx_wall1.max_index[i] != -1)){
         cur_node = node_array[i];
         auto edge2 = graph_slam->add_se3_wall_plane_edge1(cur_node, wall_plane_node1, measurement_wall, information_wall1);
        //std::cout<<"Edge IDs Wall1:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
        }
     }
     if(W2){
       if((plane_idx_wall2.min_index[i] != -1) && (plane_idx_wall2.max_index[i] != -1)){
         cur_node = node_array[i];
         auto edge2 = graph_slam->add_se3_wall_plane_edge2(cur_node, wall_plane_node2, measurement_wall, information_wall2);
        //std::cout<<"Edge IDs Wall2:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
        }
     }
     if(W3){
       if((plane_idx_wall3.min_index[i] != -1) && (plane_idx_wall3.max_index[i] != -1)){
         cur_node = node_array[i];
         auto edge3 = graph_slam->add_se3_wall_plane_edge3(cur_node, wall_plane_node3, measurement_wall, information_wall3);
        //std::cout<<"Edge IDs Wall3:"<<edge3->vertices()[0]->id()<<" "<<edge3->vertices()[1]->id()<<std::endl;
        }
     }
  }  
}

//save data in a file and optimize the graph
void graph_optimization()
{
  int num_iteration = 50;
  //Save the graph before optimization
  std::string filename1 = "before_opti_graph.g2o";
  std::string filename2 = "after_opti_graph.g2o";
  graph_slam->save(filename1);
  graph_slam->optimize(num_iteration);
  graph_slam->save(filename2); 
}

//display the pose
void display_pose()
{
  g2o::VertexSE3* node;
  for(int i=0; i < node_array.size();i++)
  {
    node = node_array[i];
    Eigen::Vector3d pos = node->estimate().translation();
    std::cout<<"x: "<<pos.x()<<" y: "<<pos.y()<<" z: "<<pos.z()<<std::endl;
  }
}

//modify filterred pcd file according to final pose graph
void modified_pcd_file()
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr merge_cloud(new pcl::PointCloud<PointT>);
  g2o::VertexSE3* node;
  
  for(int i=0; i < node_array.size();i++)
  {
    extract_cloud(frame_cloud, i);
    node = node_array[i];
    Eigen::Isometry3d transform_mat = node->estimate();
    trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
    merge_point_cloud(transformed_cloud, merge_cloud, i);
    //std::cout<<"Matrix: "<<transform_mat.matrix()<<std::endl;
  }
  pcl::io::savePCDFileASCII ("merged_pcd.pcd", *merge_cloud);
  std::cerr << "Saved " << merge_cloud->points.size () << " data points to merged_pcd.pcd." << std::endl;
}

//extract the frame cloud data from the filttered cloud
void extract_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, int vertexID)
{
  frame_cloud->width    = filterred_idx.max_index[vertexID] - filterred_idx.min_index[vertexID] + 1;
  frame_cloud->height   = 1;
  frame_cloud->is_dense = false;
  frame_cloud->points.resize (frame_cloud->width * frame_cloud->height);

  for (size_t i = 0; i < frame_cloud->points.size (); ++i){
    frame_cloud->points[i] = filterred_data->points[filterred_idx.min_index[vertexID]+i];
  }

}

//merge the transformed frame cloud data into merge_cloud
void merge_point_cloud(pcl::PointCloud<PointT>::Ptr transformed_cloud, pcl::PointCloud<PointT>::Ptr merge_cloud, int vertexID)
{
  merge_cloud->width    = filterred_idx.max_index[vertexID] + 1;
  merge_cloud->height   = 1;
  merge_cloud->is_dense = false;
  merge_cloud->points.resize (merge_cloud->width * merge_cloud->height);
  size_t t = 0;
  for (size_t i = filterred_idx.min_index[vertexID]; i <= filterred_idx.max_index[vertexID]; ++i){
    merge_cloud->points[i] = transformed_cloud->points[t];
    t++;
  }

}

//Calculate the Global RSS distance for plane points to the plane
double calc_globalRSS_after_optimization(pcl::PointCloud<PointT>::Ptr plane_pts, key_index plane_index, Eigen::VectorXf coeff, std::vector<double> &distances)
{
  double global_rss = 0.0;
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  g2o::VertexSE3* node;
  std::vector<double> frame_pts_distances;
  
  for(int i=0; i < node_array.size();i++)
  {
    if((plane_index.min_index[i] == -1) && (plane_index.max_index[i] == -1))
      continue;
    else{
      extract_frame_cloud(frame_cloud, plane_pts, plane_index, i+1);
      node = node_array[i];
      Eigen::Isometry3d transform_mat = node->estimate();
      trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat); 
      compute_distance(transformed_cloud, coeff, frame_pts_distances);
      //compute the rss distance
      double dist = 0.0;
      for(size_t j = 0; j<(plane_index.max_index[i] - plane_index.min_index[i] + 1); j++){
        dist+= pow(frame_pts_distances[j],2);
        distances.push_back(frame_pts_distances[j]);
      }
     global_rss += sqrt(dist);
    }
  }
  
  return global_rss;
}
