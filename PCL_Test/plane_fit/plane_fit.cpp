#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

int main (int argc, char** argv)
{
  typedef pcl::PointXYZI PointT;
  // initialize PointClouds
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr final (new pcl::PointCloud<PointT>);


  if (pcl::io::loadPCDFile<PointT> ("filtered_data.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width <<"x"<< cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  
  std::vector<int> inliers;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT> (cloud));
  pcl::RandomSampleConsensus<PointT> ransac (model_p);
  ransac.setDistanceThreshold (.02);
  ransac.computeModel();
  ransac.getInliers(inliers);
      
  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud (*cloud, inliers, *final);
  pcl::io::savePCDFileASCII ("test_pcd.pcd", *final);
  std::cerr << "Saved " << final->points.size () << " data points to test_pcd.pcd." << std::endl;

  //Compute the distances to the model
  //get the model coefficients first
  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);
  
  std::vector<double> distances;
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_inliers(new pcl::SampleConsensusModelPlane<PointT> (final));
  model_inliers->getDistancesToModel(coeffs, distances);
  //display some distance data  
  for(int i=0;i<50;i++)
    std::cout<<"Distances: "<<distances[i]<<std::endl;
  std::cout<<"Data size: "<<distances.size();
  // creates the visualization object and adds either our original cloud or all of the inliers
  // depending on the command line arguments specified.
  //pcl::visualization::PCLVisualizer::Ptr viewer;
  //viewer = simpleVis(final);
  //viewer = simpleVis(cloud);
  //while (!viewer->wasStopped ())
  //{
    //viewer->spinOnce (100);
    //std::this_thread::sleep_for(100ms);
  //}
  return 0;
}
