#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/time.h>
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"


typedef pcl::PointXYZI PointT;

int
main(int argc, char** argv)
{
    
    std::vector<int> indices;

    pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
    
     //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read("original_pcd.pcd", *input_cloud);
    //Remove NaN data from the input cloud
    pcl::removeNaNFromPointCloud(*input_cloud, *cloud_filtered, indices);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    // Search for a plane perpendicular to some axis (specified below).
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // Set the distance to the plane for a point to be an inlier.
    seg.setDistanceThreshold(0.60);   

    // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
    //Eigen::Vector3f axis;
    //axis << 0, 1, 0;
    //seg.setAxis(axis);
    //seg.setEpsAngle(pcl::deg2rad(20.0));
    seg.setMaxIterations (1000);
     
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    
    int i = 0, nr_points = (int)cloud_filtered->points.size();
    
    // While 10% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.1 * nr_points)
    {
    	// Segment the largest planar component from the remaining cloud
    	seg.setInputCloud (cloud_filtered);
    	seg.segment (*inliers, *coeff);
    	if (inliers->indices.size () == 0)
    	{
      		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      		break;
   	}

    	// Extract the inliers
    	extract.setInputCloud (cloud_filtered);
    	extract.setIndices (inliers);
   	extract.setNegative (false);
    	extract.filter (*cloud_p);
    	std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    	std::stringstream ss;
    	ss << "new_pcl" << i << ".pcd";
    	//writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
	pcl::io::savePCDFileASCII(ss.str (), *cloud_p);

    	// Create the filtering object
    	extract.setNegative (true);
    	extract.filter (*cloud_f);
    	cloud_filtered.swap (cloud_f);
    	i++;
  }
    return (0);
}
