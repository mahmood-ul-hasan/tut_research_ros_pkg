#include<graph_optimization.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include<pre_processing.hpp>

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
#include <g2o/edge_se3_wall_plane3.hpp>
#include <g2o/edge_se3_wall_plane4.hpp>
#include <g2o/edge_se3_wall_plane5.hpp>

#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;

std::unique_ptr<GraphSLAM> graph_slam;
std::vector<g2o::VertexSE3*> node_array;

void graph_construction(Eigen::VectorXf &floor_coeffs)
{
  g2o::VertexSE3* prev_node, *cur_node; 
  g2o::VertexPlane* floor_plane_node;
  g2o::VertexPlane* wall_plane_node1;
  g2o::VertexPlane* wall_plane_node2;
  g2o::VertexPlane* wall_plane_node3;
  g2o::VertexPlane* wall_plane_node4;
  g2o::VertexPlane* wall_plane_node5;

  std::unique_ptr<InformationMatrixCalculator> info_calculator;

  double floor_edge_stddev = 10;
  double wall_edge_stddev1 = 20;
  double wall_edge_stddev2 = 40;
  double wall_edge_stddev3 = 40;
  double wall_edge_stddev4 = 40;
  double wall_edge_stddev5 = 40;
  
  double floor_edge_robust_kernel_size = 1.0;
  std::string floor_edge_robust_kernel ="NONE";
  //double node_edge_robust_kernel_size = 1.0;
  //std::string node_edge_robust_kernel ="Huber";
  floor_plane_node = nullptr;
  wall_plane_node1 = nullptr;
  wall_plane_node2 = nullptr;
  wall_plane_node3 = nullptr;
  wall_plane_node4 = nullptr;
  wall_plane_node5 = nullptr;

  //Initialize
  graph_slam.reset(new GraphSLAM("lm_var"));
  info_calculator.reset(new InformationMatrixCalculator());

  //Add floor plane node
  floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(floor_coeffs[0], floor_coeffs[1], floor_coeffs[2], floor_coeffs[3]));//0.0, 0.0, 1.0, 0.0));
  floor_plane_node->setFixed(true);
  
  
  Eigen::Isometry3d sen_pose = Eigen::Isometry3d::Identity();
  prev_node = graph_slam->add_se3_node(sen_pose);
  node_array.push_back(prev_node);
  
  //Create and set relative pose to 0
  Eigen::Isometry3d rel_pose;
  Eigen::Matrix4d x = Eigen::Matrix4d::Identity();
  rel_pose.matrix() = x;
  
  //Add an edge between floor plan node and first pose node
  //Eigen::Vector4d coeffs(floor_coeffs[0], floor_coeffs[1], floor_coeffs[2], floor_coeffs[3]);
  Eigen::Vector4d measurement(0,0,0,0);
  Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
  information(0,0) = 1.0 / floor_edge_stddev;
  //information(1,1) = 0;//1.0 / floor_edge_stddev;
  //information(2,2) = 0;//1.0 / floor_edge_stddev;
  //std::cout<<"Floor Coeffs: "<<floor_coeffs<<std::endl;

  auto edge = graph_slam->add_se3_ground_plane_edge(prev_node, floor_plane_node, measurement, information);
  graph_slam->add_robust_kernel(edge, floor_edge_robust_kernel, floor_edge_robust_kernel_size);
  double rel_distance[1];
  Eigen::MatrixXd info_mat;
  //info_mat = info_calculator->calc_const_information_matrix();
  //std::cout<<"Edge Info. Matrix: "<<info_mat<<std::endl;
  
  //Add the successive nodes and edges to the graph using the sensor pose and constant information matrix
  for(int i = 1; i < plane_idx.frame_id.size(); i++)
  {
    cur_node = graph_slam->add_se3_node(sen_pose);
    node_array.push_back(cur_node);
    //rel_distance[0] = (frame_rms_distances[i-1] + frame_rms_distances[i])/2.0;
    info_mat = info_calculator->calc_const_information_matrix();
    //std::cout << "Info_Mat: "<<info_mat << std::endl;	
    auto edge1 = graph_slam->add_se3_edge(prev_node, cur_node, rel_pose, info_mat);
    //graph_slam->add_robust_kernel(edge1, node_edge_robust_kernel, node_edge_robust_kernel_size);
    auto edge2 = graph_slam->add_se3_ground_plane_edge(cur_node, floor_plane_node, Eigen::Vector4d(0,0,0,0), information);
    //graph_slam->add_robust_kernel(edge2, floor_edge_robust_kernel, floor_edge_robust_kernel_size);
    prev_node = cur_node;
  }
 
  //add wall_plane_nodes and wall_plane_edges to the graph
  wall_plane_node1 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall1[0], coeffs_wall1[1], coeffs_wall1[2], coeffs_wall1[3]));
  //wall_plane_node1->setFixed(true);
  wall_plane_node2 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall2[0], coeffs_wall2[1], coeffs_wall2[2], coeffs_wall2[3]));
  //wall_plane_node2->setFixed(true);
  wall_plane_node3 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall3[0], coeffs_wall3[1], coeffs_wall3[2], coeffs_wall3[3]));
  //wall_plane_node3->setFixed(true);
  wall_plane_node4 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall4[0], coeffs_wall4[1], coeffs_wall4[2], coeffs_wall4[3]));
  //wall_plane_node4->setFixed(true);
  wall_plane_node5 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall5[0], coeffs_wall5[1], coeffs_wall5[2], coeffs_wall5[3]));
  //wall_plane_node5->setFixed(true);

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

  Eigen::Matrix3d information_wall4 = Eigen::Matrix3d::Identity();
  information_wall4(0,0) = 1.0 / wall_edge_stddev4;
  //information_wall4(1,1) = 0;//1.0 / wall_edge_stddev4;
  //information_wall4(2,2) = 0;//1.0 / wall_edge_stddev4;

  Eigen::Matrix3d information_wall5 = Eigen::Matrix3d::Identity();
  information_wall5(0,0) = 1.0 / wall_edge_stddev5;
  //information_wall5(1,1) = 0;//1.0 / wall_edge_stddev5;
  //information_wall5(2,2) = 0;//1.0 / wall_edge_stddev5;

  for(int i = 0; i < plane_idx_wall1.frame_id.size(); i++)
  {
     if((plane_idx_wall1.min_index[i] != -1) && (plane_idx_wall1.max_index[i] != -1)){
       cur_node = node_array[i];
       auto edge2 = graph_slam->add_se3_wall_plane_edge1(cur_node, wall_plane_node1, measurement_wall, information_wall1);
       //std::cout<<"Edge IDs Wall1:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
     }
     if((plane_idx_wall2.min_index[i] != -1) && (plane_idx_wall2.max_index[i] != -1)){
       cur_node = node_array[i];
       auto edge2 = graph_slam->add_se3_wall_plane_edge2(cur_node, wall_plane_node2, measurement_wall, information_wall2);
       //std::cout<<"Edge IDs Wall2:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
     }
     if((plane_idx_wall3.min_index[i] != -1) && (plane_idx_wall3.max_index[i] != -1)){
       cur_node = node_array[i];
       auto edge2 = graph_slam->add_se3_wall_plane_edge3(cur_node, wall_plane_node3, measurement_wall, information_wall3);
       //std::cout<<"Edge IDs Wall3:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
     }
     if((plane_idx_wall4.min_index[i] != -1) && (plane_idx_wall4.max_index[i] != -1)){
       cur_node = node_array[i];
       auto edge2 = graph_slam->add_se3_wall_plane_edge4(cur_node, wall_plane_node4, measurement_wall, information_wall4);
       //std::cout<<"Edge IDs Wall4:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
     }
     if((plane_idx_wall5.min_index[i] != -1) && (plane_idx_wall5.max_index[i] != -1)){
       cur_node = node_array[i];
       auto edge2 = graph_slam->add_se3_wall_plane_edge5(cur_node, wall_plane_node5, measurement_wall, information_wall5);
       //std::cout<<"Edge IDs Wall5:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
     }
  }
}

//save data in a file and optimize the graph
void graph_optimization()
{
  int num_iteration = 500;
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


















