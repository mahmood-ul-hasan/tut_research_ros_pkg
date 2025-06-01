#ifndef COMMON_GLOBAL_HPP
#define COMMON_GLOBAL_HPP
#include <iostream>
#include <math.h>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<key_index.hpp>


typedef pcl::PointXYZI PointT;

/*Global variables*/
extern pcl::PointCloud<PointT>::Ptr plane_points (new pcl::PointCloud<PointT>);
extern Eigen::VectorXf coeffs;
extern key_index plane_idx;

#endif // COMMON_GLOBAL_HPP


//Save coefficent, plane_idx,  and inliers_idx to a file for later use
  std::ofstream outFile;
  outFile.open ("coeffs_idx.txt", std::ofstream::out | std::ofstream::app);
  outFile << coeffs <<"\n";                         //write plane coefficient to a file
  size_t s;
  s = plane_idx.frame_id.size();
  outFile << s <<"\n";                              //write the size of plane_idx to a file
  for (size_t i = 0; i < s; i++){                   //write plane_idx to a file 
    outFile << plane_idx.frame_id[i]<<"\n";
    outFile << plane_idx.min_index[i]<<"\n";
    outFile << plane_idx.max_index[i]<<"\n";
  }
  
  s = inliers_idx.size(); 
  outFile << s <<"\n";                               //write the size of the inliers_idx size
  for (size_t i = 0; i < s; i++)                     //write inliers_idx to a file 
   outFile << inliers_idx[i]<<"\n";

  outFile.close();                                  //close the output file
 
  std::ifstream inFile;
  inFile.open ("coeffs_idx.txt");  
  Eigen::Vector4f coeffs1;
  inFile >> coeffs1(0)>> coeffs1(1)>> coeffs1(2)>> coeffs1(3);
  std::cout<< coeffs1 <<"\n";
  int size;
  inFile >> size; 
  std::cout << size <<"\n";
  int x,y,z;
  key_index plane_idx1;
  for (size_t i=0; i<size; i++){ 
    inFile >> x >> y >> z;
    plane_idx1.frame_id.push_back(x);
    plane_idx1.min_index.push_back(y);
    plane_idx1.max_index.push_back(z);

    std::cout<< plane_idx1.frame_id[i] << plane_idx1.min_index[i] << plane_idx1.max_index[i]<< "\n";
  }

  inFile >> size;
  std::cout << size <<"\n";
  std::vector<int> inliers_idx1;
  int in_idx;
  
  for (size_t i=0; i<size; i++){ 
    inFile >> in_idx;
    inliers_idx1.push_back(in_idx);
    std::cout<<inliers_idx1[i]<<" ";
  }
  inFile.close();
