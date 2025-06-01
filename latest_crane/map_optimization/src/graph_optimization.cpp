#include <graph_optimization.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include <pre_processing.hpp>
#include <hdl_graph_slam/ros_utils.hpp>

#include <iostream>
#include <cmath>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_ground_plane.hpp>
#include <g2o/edge_se3_ground_plane2.hpp>
#include <g2o/edge_se3_ground_plane3.hpp>
#include <g2o/edge_se3_ground_plane4.hpp>
#include <g2o/edge_se3_wall_plane1.hpp>
#include <g2o/edge_se3_wall_plane2.hpp>
#include <g2o/edge_se3_wall_plane3.hpp>
#include <g2o/edge_se3_wall_plane4.hpp>
#include <g2o/edge_se3_wall_plane5.hpp>

#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"

#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseArray.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>


using namespace hdl_graph_slam;
typedef pcl::PointXYZI PointT;

std::unique_ptr<GraphSLAM> graph_slam;
std::vector<g2o::VertexSE3 *> node_array;

extern double  floor_edge_stddev;
extern double floor_edge_stddev2;
extern double floor_edge_stddev3;
extern double floor_edge_stddev4;
extern double wall_edge_stddev1;
extern double wall_edge_stddev2;
extern double wall_edge_stddev3;
extern double wall_edge_stddev4;
extern double wall_edge_stddev5;
extern double plane_to_plane_edge_stddev;
extern double parallel_edge_stddev;
extern std::string build_folder;


void graph_construction(Eigen::VectorXf &floor_coeffs)
{
  g2o::VertexSE3 *prev_node, *cur_node;
  g2o::VertexPlane *floor_plane_node;
  g2o::VertexPlane *floor_plane_node2;
  g2o::VertexPlane *floor_plane_node3;
  g2o::VertexPlane *floor_plane_node4;
  g2o::VertexPlane *wall_plane_node1;
  g2o::VertexPlane *wall_plane_node2;
  g2o::VertexPlane *wall_plane_node3;
  g2o::VertexPlane *wall_plane_node4;
  g2o::VertexPlane *wall_plane_node5;

  std::unique_ptr<InformationMatrixCalculator> info_calculator;

 //double node_edge_robust_kernel_size = 1.0;
  //std::string node_edge_robust_kernel ="Huber";
  floor_plane_node = nullptr;
  floor_plane_node2 = nullptr;
  floor_plane_node3 = nullptr;
  floor_plane_node4 = nullptr;
  wall_plane_node1 = nullptr;
  wall_plane_node2 = nullptr;
  wall_plane_node3 = nullptr;
  wall_plane_node4 = nullptr;
  wall_plane_node5 = nullptr;


  std::cout << "Initialize" << std::endl;
//Initialize
  graph_slam.reset(new GraphSLAM("lm_var"));
  info_calculator.reset(new InformationMatrixCalculator());


  //----------------------------------------
  // parameters adjustment
  std::cout << "parameters adjustment" << std::endl;

  Eigen::MatrixXd info_mat;
  // info_mat = info_calculator->calc_const_information_matrix();
  // std::cout << "Info. Matrix for pose to pose edges: \n" << info_mat << std::endl;

  std::cout << "floor_edge_stddev " << floor_edge_stddev << std::endl;
  std::cout << "floor_edge_stddev " << floor_edge_stddev2 << std::endl;
  std::cout << "floor_edge_stddev " << floor_edge_stddev3 << std::endl;
  std::cout << "floor_edge_stddev " << floor_edge_stddev4 << std::endl;

  std::cout << "wall_edge_stddev1 " << wall_edge_stddev1 << std::endl;
  std::cout << "wall_edge_stddev1 " << wall_edge_stddev2 << std::endl;
  std::cout << "wall_edge_stddev1 " << wall_edge_stddev3 << std::endl;
  std::cout << "wall_edge_stddev1 " << wall_edge_stddev4 << std::endl;
  std::cout << "wall_edge_stddev1 " << wall_edge_stddev5 << std::endl;

  double floor_edge_robust_kernel_size = 1.0;
  std::string floor_edge_robust_kernel = "NONE";

  std::cout << "information matrix" << std::endl;

  Eigen::Matrix3d information_ground1 = Eigen::Matrix3d::Identity();
  information_ground1(0, 0) = 1.0 / floor_edge_stddev;
  information_ground1(1,1) = 1.0 / floor_edge_stddev;
  information_ground1(2,2) = 1.0 / floor_edge_stddev;

  Eigen::Matrix3d information_ground2 = Eigen::Matrix3d::Identity();
  information_ground2(0, 0) = 1.0 / floor_edge_stddev2;
  information_ground2(1,1) = 1.0 / floor_edge_stddev2;
  information_ground2(2,2) = 1.0 / floor_edge_stddev2;


  Eigen::Matrix3d information_ground3 = Eigen::Matrix3d::Identity();
  information_ground3(0, 0) = 1.0 / floor_edge_stddev3;
  information_ground3(1,1) = 1.0 / floor_edge_stddev3;
  information_ground3(2,2) = 1.0 / floor_edge_stddev3;

  Eigen::Matrix3d information_ground4 = Eigen::Matrix3d::Identity();
  information_ground4(0, 0) = 1.0 / floor_edge_stddev4;
  information_ground4(1,1) = 1.0 / floor_edge_stddev4;
  information_ground4(2,2) = 1.0 / floor_edge_stddev4;

  Eigen::Matrix3d information_wall1 = Eigen::Matrix3d::Identity();
  information_wall1(0, 0) = 1.0 / wall_edge_stddev1;
  information_wall1(1,1) = 1.0 / wall_edge_stddev1;
  information_wall1(2,2) = 1.0 / wall_edge_stddev1;

  Eigen::Matrix3d information_wall2 = Eigen::Matrix3d::Identity();
  information_wall2(0, 0) = 1.0 / wall_edge_stddev2;
  information_wall2(1,1) = 1.0 / wall_edge_stddev2;
  information_wall2(2,2) = 1.0 / wall_edge_stddev2;

  Eigen::Matrix3d information_wall3 = Eigen::Matrix3d::Identity();
  information_wall3(0, 0) = 1.0 / wall_edge_stddev3;
  information_wall3(1,1) = 1.0 / wall_edge_stddev3;
  information_wall3(2,2) = 1.0 / wall_edge_stddev3;

  Eigen::Matrix3d information_wall4 = Eigen::Matrix3d::Identity();
  information_wall4(0, 0) = 1.0 / wall_edge_stddev4;
  information_wall4(1,1) = 1.0 / wall_edge_stddev4;
  information_wall4(2,2) = 1.0 / wall_edge_stddev4;

  Eigen::Matrix3d information_wall5 = Eigen::Matrix3d::Identity();
  information_wall5(0, 0) = 1.0 / wall_edge_stddev5;
  information_wall5(1,1) = 1.0 / wall_edge_stddev5;
  information_wall5(2,2) = 1.0 / wall_edge_stddev5;

  Eigen::Matrix4d information_plane_to_plane_edge = Eigen::Matrix4d::Identity();
  information_plane_to_plane_edge(0, 0) = 1.0 / plane_to_plane_edge_stddev;
  information_plane_to_plane_edge(1, 1) = 1.0 / plane_to_plane_edge_stddev;
  information_plane_to_plane_edge(2, 2) = 1.0 / plane_to_plane_edge_stddev;
  information_plane_to_plane_edge(3, 3) = 1.0 / plane_to_plane_edge_stddev;

Eigen::Matrix3d information_parallel_edge = Eigen::Matrix3d::Identity();
information_parallel_edge(0,0)= 1.0 / parallel_edge_stddev;
information_parallel_edge(1,1)= 1.0 / parallel_edge_stddev;
information_parallel_edge(2,2)= 1.0 / parallel_edge_stddev;
  Eigen::Vector4d measurement(0, 0, 0, 0);
  Eigen::Vector4d measurement_wall(0, 0, 0, 0);
  Eigen::Vector4d measurement_plane_to_plane_edge(0, 0, 0, 0);
  Eigen::Vector3d measurement_parallel_edge(0, 0, 0);

 

  std::cout << "ground node" << coeffs_ground[0] << " "<< coeffs_ground[1]<< " " << coeffs_ground[2] <<" "<< coeffs_ground[3]<<std::endl;


  //Add floor plane node
  floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_ground1[0], coeffs_ground1[1], coeffs_ground1[2], coeffs_ground1[3])); //0.0, 0.0, 1.0, 0.0));
  // floor_plane_node->setFixed(true);

  Eigen::Isometry3d sen_pose = Eigen::Isometry3d::Identity();
  prev_node = graph_slam->add_se3_node(sen_pose);
  node_array.push_back(prev_node);

  //Create and set relative pose to 0
  Eigen::Isometry3d rel_pose;
  Eigen::Matrix4d x = Eigen::Matrix4d::Identity();
  rel_pose.matrix() = x;

  //Add an edge between floor plan node and first pose node
  auto edge_g1 = graph_slam->add_se3_ground_plane_edge(prev_node, floor_plane_node, measurement, information_ground1);
  graph_slam->add_robust_kernel(edge_g1, floor_edge_robust_kernel, floor_edge_robust_kernel_size);

  double rel_distance[1];

  //Add the successive nodes and edges to the graph using the sensor pose and constant information matrix

    std::cout << "loop for pose node and edges" << std::endl;
  
  pcl::PointCloud<PointT>::Ptr frame_cloud_prev(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr frame_cloud_curr(new pcl::PointCloud<PointT>);

    info_mat = info_calculator->calc_const_information_matrix();
        // std::cout << "Info. Matrix for pose to pose edges: \n" << info_mat << std::endl;    

  for (int i = 1; i < plane_idx_ground1.frame_id.size(); i++)
  {
    cur_node = graph_slam->add_se3_node(sen_pose);
    node_array.push_back(cur_node);



    auto edge1 = graph_slam->add_se3_edge(prev_node, cur_node, rel_pose, info_mat);
    //graph_slam->add_robust_kernel(edge1, node_edge_robust_kernel, node_edge_robust_kernel_size);

    if ((plane_idx_ground1.min_index[i] != -1) && (plane_idx_ground1.max_index[i] != -1))
    {
      auto edge_g1 = graph_slam->add_se3_ground_plane_edge(cur_node, floor_plane_node, Eigen::Vector4d(0, 0, 0, 0), information_ground1);
      //graph_slam->add_robust_kernel(edge2, floor_edge_robust_kernel, floor_edge_robust_kernel_size);
      edges_ground1.push_back(edge_g1->vertices()[0]->id());
      node_ground1 = edge_g1->vertices()[1]->id();
    }
    prev_node = cur_node;
    // frame_cloud_prev = frame_cloud_curr;
  }

    std::cout << "loop for plane node" << std::endl;

if (number_of_ground_planes >= 2) {
  floor_plane_node2 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_ground2[0], coeffs_ground2[1], coeffs_ground2[2], coeffs_ground2[3]));
}
if (number_of_ground_planes >= 3) {
floor_plane_node3 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_ground3[0], coeffs_ground3[1], coeffs_ground3[2], coeffs_ground3[3]));
}

if (number_of_ground_planes >= 4) {
floor_plane_node4 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_ground4[0], coeffs_ground4[1], coeffs_ground4[2], coeffs_ground4[3]));
}

  //add wall_plane_nodes and wall_plane_edges to the graph
  wall_plane_node1 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall1[0], coeffs_wall1[1], coeffs_wall1[2], coeffs_wall1[3]));
  //wall_plane_node1->setFixed(true);
    if (number_of_wall_planes >= 2) {
  wall_plane_node2 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall2[0], coeffs_wall2[1], coeffs_wall2[2], coeffs_wall2[3]));
  //wall_plane_node2->setFixed(true);
    }
    if (number_of_wall_planes >= 3) {
  wall_plane_node3 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall3[0], coeffs_wall3[1], coeffs_wall3[2], coeffs_wall3[3]));
  //wall_plane_node3->setFixed(true);
     }
    if (number_of_wall_planes >= 4) {
  wall_plane_node4 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall4[0], coeffs_wall4[1], coeffs_wall4[2], coeffs_wall4[3]));
  //wall_plane_node4->setFixed(true);
     }
    if (number_of_wall_planes >= 5) {
  wall_plane_node5 = graph_slam->add_plane_node(Eigen::Vector4d(coeffs_wall5[0], coeffs_wall5[1], coeffs_wall5[2], coeffs_wall5[3]));
  //wall_plane_node5->setFixed(true);
  }



    // add_plane_identity_edge
  // if (number_of_ground_planes >= 2) {
  // auto edge_p_p1 = graph_slam->add_plane_identity_edge(floor_plane_node, floor_plane_node2, measurement_plane_to_plane_edge, information_plane_to_plane_edge);
  //   }
  //   if (number_of_ground_planes >= 3) {
  //  auto edge_p_p2 = graph_slam->add_plane_identity_edge(floor_plane_node2, floor_plane_node3, measurement_plane_to_plane_edge, information_plane_to_plane_edge);
  // auto edge_p_p3 = graph_slam->add_plane_identity_edge(floor_plane_node3, floor_plane_node, measurement_plane_to_plane_edge, information_plane_to_plane_edge);
  //       }
  // add_plane_parallel_edge
  // auto edge_p_w1 = graph_slam->add_plane_parallel_edge(floor_plane_node, wall_plane_node4, measurement_parallel_edge, information_parallel_edge);
  // auto edge_p_w2 = graph_slam->add_plane_parallel_edge(floor_plane_node, wall_plane_node5, measurement_parallel_edge, information_parallel_edge);




    std::cout << "loop for plane edges" << std::endl;

  for (int i = 0; i < plane_idx_wall1.frame_id.size(); i++)
  {

    if (number_of_ground_planes >= 2 && (plane_idx_ground2.min_index[i] != -1) && (plane_idx_ground2.max_index[i] != -1))
    {
      cur_node = node_array[i];
      auto edge_g2 = graph_slam->add_se3_ground_plane_edge2(cur_node, floor_plane_node2, measurement_wall, information_ground2);
      // std::cout<<"Edge IDs Wall1:"<<edge_g2->vertices()[0]->id()<<" "<<edge_g2->vertices()[1]->id()<<std::endl;
      edges_ground2.push_back(edge_g2->vertices()[0]->id());
      node_ground2 = edge_g2->vertices()[1]->id();
    }

    if (number_of_ground_planes >= 3 && ( plane_idx_ground3.min_index[i] != -1) && (plane_idx_ground3.max_index[i] != -1))
    {
      cur_node = node_array[i];
      auto edge_g3 = graph_slam->add_se3_ground_plane_edge3(cur_node, floor_plane_node3, measurement_wall, information_ground3);
      // std::cout<<"Edge IDs ground3:"<<edge_g3->vertices()[0]->id()<<" "<<edge_g3->vertices()[1]->id()<<std::endl;
      edges_ground3.push_back(edge_g3->vertices()[0]->id());
      node_ground3 = edge_g3->vertices()[1]->id();
    }


    if (number_of_ground_planes >= 4 && ( plane_idx_ground3.min_index[i] != -1) && (plane_idx_ground3.max_index[i] != -1))
    {
      cur_node = node_array[i];
      auto edge_g4 = graph_slam->add_se3_ground_plane_edge4(cur_node, floor_plane_node4, measurement_wall, information_ground4);
      // std::cout<<"Edge IDs ground3:"<<edge_g3->vertices()[0]->id()<<" "<<edge_g3->vertices()[1]->id()<<std::endl;
      edges_ground4.push_back(edge_g4->vertices()[0]->id());
      node_ground4 = edge_g4->vertices()[1]->id();
    }

    if (number_of_wall_planes >= 1 && (plane_idx_wall1.min_index[i] != -1) && (plane_idx_wall1.max_index[i] != -1))
    {
      cur_node = node_array[i];
      auto edge_w1 = graph_slam->add_se3_wall_plane_edge1(cur_node, wall_plane_node1, measurement_wall, information_wall1);
      //std::cout<<"Edge IDs Wall1:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
      edges_wall1.push_back(edge_w1->vertices()[0]->id());
      node_wall1 = edge_w1->vertices()[1]->id();

    }
    if (number_of_wall_planes >= 2 && (plane_idx_wall2.min_index[i] != -1) && (plane_idx_wall2.max_index[i] != -1) )
    {
      cur_node = node_array[i];
      auto edge_w2 = graph_slam->add_se3_wall_plane_edge2(cur_node, wall_plane_node2, measurement_wall, information_wall2);
      edges_wall2.push_back(edge_w2->vertices()[0]->id());
      node_wall2 = edge_w2->vertices()[1]->id();

      //std::cout<<"Edge IDs Wall2:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
    }
    if (number_of_wall_planes >= 3  &&  (plane_idx_wall3.min_index[i] != -1) && (plane_idx_wall3.max_index[i] != -1) && number_of_wall_planes >= 3)
    {
      cur_node = node_array[i];
      auto edge_w3 = graph_slam->add_se3_wall_plane_edge3(cur_node, wall_plane_node3, measurement_wall, information_wall3);
      edges_wall3.push_back(edge_w3->vertices()[0]->id());
      node_wall3 = edge_w3->vertices()[1]->id();

      //std::cout<<"Edge IDs Wall3:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
    }
    if (number_of_wall_planes >= 4  &&  (plane_idx_wall4.min_index[i] != -1) && (plane_idx_wall4.max_index[i] != -1) && number_of_wall_planes >= 4)
    {
      cur_node = node_array[i];
      auto edge_w4 = graph_slam->add_se3_wall_plane_edge4(cur_node, wall_plane_node4, measurement_wall, information_wall4);
      edges_wall4.push_back(edge_w4->vertices()[0]->id());
      node_wall4 = edge_w4->vertices()[1]->id();

      //std::cout<<"Edge IDs Wall4:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
    }
    if (number_of_wall_planes >= 5  &&  (plane_idx_wall5.min_index[i] != -1) && (plane_idx_wall5.max_index[i] != -1) && number_of_wall_planes >= 5)
    {
      cur_node = node_array[i];
      auto edge_w5 = graph_slam->add_se3_wall_plane_edge5(cur_node, wall_plane_node5, measurement_wall, information_wall5);
      edges_wall5.push_back(edge_w5->vertices()[0]->id());
      node_wall5 = edge_w5->vertices()[1]->id();
      //std::cout<<"Edge IDs Wall5:"<<edge2->vertices()[0]->id()<<" "<<edge2->vertices()[1]->id()<<std::endl;
    }
  }
}

//=====================================================================
//save data in a file and optimize the graph
//=====================================================================
void graph_optimization(int num_iteration)
{
  std::cout << "graph _optimization: " << std::endl;

  // int num_iteration = 500;
  //Save the graph before optimization

  std::string filename1 = build_folder + "before_opti_graph.g2o";
  std::string filename2 = build_folder +"after_opti_graph.g2o";
  std::cout << "graph _optimi0_slam->optimize(num_iteration)";

  graph_slam->save(filename1);

  graph_slam->optimize(num_iteration);
  graph_slam->save(filename2);
}

//====================================================================
//display
//=====================================================================
void display_pose(ros::Publisher pose_array_pub)
{
  g2o::VertexSE3 *node;
  geometry_msgs::Pose corr_pose;
  nav_msgs::Odometry odom;
  geometry_msgs::PoseArray pose_array;
  double roll, pitch, yaw;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "laser";

  for (int i = 0; i < node_array.size(); i++)
  {
    node = node_array[i];
    Eigen::Isometry3d transform_mat = node->estimate();

    //static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id)
    // trans_odom = matrix2transform(ros::Time::now(), transform_mat.matrix().cast<float>(), "world", "laser" );
    corr_pose = isometry2pose(transform_mat);

    pose_array.poses.push_back(corr_pose);

    x.push_back(corr_pose.position.x);
    y.push_back(corr_pose.position.y);
    z.push_back(corr_pose.position.z);

    tf::Quaternion q(corr_pose.orientation.x, corr_pose.orientation.y,
                     corr_pose.orientation.z, corr_pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    Roll.push_back(roll * 180 / M_PI);
    Pitch.push_back(pitch * 180 / M_PI);
    Yaw.push_back(yaw * 180 / M_PI);

    // Eigen::Vector3d pos = node->estimate().translation();
    // Eigen::Matrix3d rot =  node->estimate().rotation();
  }

  if (pose_array_pub.getNumSubscribers())
  {
    pose_array_pub.publish(pose_array);
  }
}

//modify filterred pcd file according to final pose graph
pcl::PointCloud<PointT>::Ptr modified_pcd_file()
{
  pcl::PointCloud<PointT>::Ptr frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr merge_cloud(new pcl::PointCloud<PointT>);
  g2o::VertexSE3 *node;

  for (int i = 0; i < node_array.size(); i++)
  {
    extract_cloud(frame_cloud, i);
    node = node_array[i];
    Eigen::Isometry3d transform_mat = node->estimate();
    trnasform_point_cloud(frame_cloud, transformed_cloud, transform_mat);
    merge_point_cloud(transformed_cloud, merge_cloud, i);
    //std::cout<<"Matrix: "<<transform_mat.matrix()<<std::endl;
  }
  pcl::io::savePCDFileASCII("merged_pcd.pcd", *merge_cloud);
  std::cerr << "Saved " << merge_cloud->points.size() << " data points to merged_pcd.pcd." << std::endl;
  return merge_cloud;
}


//==================================================================================================
pcl::PointCloud<pcl::PointXYZ>::Ptr smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);
    mls.setPolynomialFit (true);
  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.6);
  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
  mls.setUpsamplingRadius(0.01);
  mls.setUpsamplingStepSize(0.01);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  pcl::io::savePCDFile ("smooth_cloud.pcd", mls_points);

pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZ>); 
mls_cloud->resize(mls_points.size());

for (size_t i = 0; i < mls_points.points.size(); ++i) 
{ 
    mls_cloud->points[i].x=mls_points.points[i].x; //error 
    mls_cloud->points[i].y=mls_points.points[i].y; //error 
    mls_cloud->points[i].z=mls_points.points[i].z; //error 
}
return mls_cloud;
}




//extract the frame cloud data from the filttered cloud
void extract_cloud(pcl::PointCloud<PointT>::Ptr frame_cloud, int vertexID)
{
  frame_cloud->width = filterred_idx.max_index[vertexID] - filterred_idx.min_index[vertexID] + 1;
  frame_cloud->height = 1;
  frame_cloud->is_dense = false;
  frame_cloud->points.resize(frame_cloud->width * frame_cloud->height);

  for (size_t i = 0; i < frame_cloud->points.size(); ++i)
  {
    frame_cloud->points[i] = filterred_data->points[filterred_idx.min_index[vertexID] + i];
  }
}

//merge the transformed frame cloud data into merge_cloud
void merge_point_cloud(pcl::PointCloud<PointT>::Ptr transformed_cloud, pcl::PointCloud<PointT>::Ptr merge_cloud, int vertexID)
{
  merge_cloud->width = filterred_idx.max_index[vertexID] + 1;
  merge_cloud->height = 1;
  merge_cloud->is_dense = false;
  merge_cloud->points.resize(merge_cloud->width * merge_cloud->height);
  size_t t = 0;
  for (size_t i = filterred_idx.min_index[vertexID]; i <= filterred_idx.max_index[vertexID]; ++i)
  {
    merge_cloud->points[i] = transformed_cloud->points[t];
    t++;
  }
}

/**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
visualization_msgs::MarkerArray create_marker_array(const ros::Time &stamp)
{
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(2);
  geometry_msgs::Pose corr_pose;

  // node markers
  visualization_msgs::Marker &traj_marker = markers.markers[0];
  traj_marker.header.frame_id = "laser";
  traj_marker.header.stamp = stamp;
  traj_marker.ns = "nodes";
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::Marker::CUBE_LIST;

  traj_marker.pose.orientation.w = 1.0;
  traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = .01;

  traj_marker.points.resize(node_array.size());
  traj_marker.colors.resize(node_array.size());

  for (int i = 0; i < node_array.size(); i++)
  {
    Eigen::Vector3d pos = node_array[i]->estimate().translation();
    traj_marker.points[i].x = pos.x();
    traj_marker.points[i].y = pos.y();
    traj_marker.points[i].z = pos.z();

    // Eigen::Isometry3d transform_mat = node_array[i]->estimate();
    // corr_pose = isometry2pose(transform_mat);
    // traj_marker.pose = corr_pose;

    double p = static_cast<double>(i) / node_array.size();
    traj_marker.colors[i].r = 1.0 - p;
    traj_marker.colors[i].g = p;
    traj_marker.colors[i].b = 0.0;
    traj_marker.colors[i].a = 1.0;
  }

  // edge markers
  visualization_msgs::Marker &edge_marker = markers.markers[1];
  edge_marker.header.frame_id = "laser";
  edge_marker.header.stamp = stamp;
  edge_marker.ns = "edges";
  edge_marker.id = 1;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;

  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = edge_marker.scale.y = edge_marker.scale.z = 5;

  edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
  edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

  auto edge_itr = graph_slam->graph->edges().begin();
  for (int i = 0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++)
  {
    g2o::HyperGraph::Edge *edge = *edge_itr;
    g2o::EdgeSE3 *edge_se3 = dynamic_cast<g2o::EdgeSE3 *>(edge);
    if (edge_se3)
    {
      g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_se3->vertices()[0]);
      g2o::VertexSE3 *v2 = dynamic_cast<g2o::VertexSE3 *>(edge_se3->vertices()[1]);
      Eigen::Vector3d pt1 = v1->estimate().translation();
      Eigen::Vector3d pt2 = v2->estimate().translation();

      edge_marker.points[i * 2].x = pt1.x();
      edge_marker.points[i * 2].y = pt1.y();
      edge_marker.points[i * 2].z = pt1.z();
      edge_marker.points[i * 2 + 1].x = pt2.x();
      edge_marker.points[i * 2 + 1].y = pt2.y();
      edge_marker.points[i * 2 + 1].z = pt2.z();

      double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
      double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
      edge_marker.colors[i * 2].r = 1.0 - p1;
      edge_marker.colors[i * 2].g = p1;
      edge_marker.colors[i * 2].a = 1.0;
      edge_marker.colors[i * 2 + 1].r = 1.0 - p2;
      edge_marker.colors[i * 2 + 1].g = p2;
      edge_marker.colors[i * 2 + 1].a = 1.0;

      if (std::abs(v1->id() - v2->id()) > 2)
      {
        edge_marker.points[i * 2].z += 0.5;
        edge_marker.points[i * 2 + 1].z += 0.5;
      }

      continue;
    }


    g2o::EdgeSE3PriorXY *edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY *>(edge);
    if (edge_priori_xy)
    {
      g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_priori_xy->vertices()[0]);
      Eigen::Vector3d pt1 = v1->estimate().translation();
      Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
      pt2.head<2>() = edge_priori_xy->measurement();

      edge_marker.points[i * 2].x = pt1.x();
      edge_marker.points[i * 2].y = pt1.y();
      edge_marker.points[i * 2].z = pt1.z() + 0.5;
      edge_marker.points[i * 2 + 1].x = pt2.x();
      edge_marker.points[i * 2 + 1].y = pt2.y();
      edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

      edge_marker.colors[i * 2].r = 1.0;
      edge_marker.colors[i * 2].a = 1.0;
      edge_marker.colors[i * 2 + 1].r = 1.0;
      edge_marker.colors[i * 2 + 1].a = 1.0;

      continue;
    }

    g2o::EdgeSE3PriorXYZ *edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ *>(edge);
    if (edge_priori_xyz)
    {
      g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_priori_xyz->vertices()[0]);
      Eigen::Vector3d pt1 = v1->estimate().translation();
      Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

      edge_marker.points[i * 2].x = pt1.x();
      edge_marker.points[i * 2].y = pt1.y();
      edge_marker.points[i * 2].z = pt1.z() + 0.5;
      edge_marker.points[i * 2 + 1].x = pt2.x();
      edge_marker.points[i * 2 + 1].y = pt2.y();
      edge_marker.points[i * 2 + 1].z = pt2.z();

      edge_marker.colors[i * 2].r = 1.0;
      edge_marker.colors[i * 2].a = 1.0;
      edge_marker.colors[i * 2 + 1].r = 1.0;
      edge_marker.colors[i * 2 + 1].a = 1.0;

      continue;
    }
  }

  return markers;
}
