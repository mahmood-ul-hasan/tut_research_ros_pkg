#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud7 (new pcl::PointCloud<pcl::PointXYZ>);


  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Trial11_17/pcdT11.pcd", *cloud1) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcdT11.pcd \n");
    return (-1);
  }
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Trial11_17/pcdT12.pcd", *cloud2) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcdT12.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Trial11_17/pcdT13.pcd", *cloud3) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcdT13.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Trial11_17/pcdT14.pcd", *cloud4) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcdT14.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Trial11_17/pcdT15.pcd", *cloud5) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcdT15.pcd.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Trial11_17/pcdT16.pcd", *cloud6) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcdT16.pcd.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Trial11_17/pcdT17.pcd", *cloud7) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcdT17.pcd \n");
    return (-1);
  }

  // Fill in the cloud data
  cloud.width    = cloud1->width + cloud2->width + cloud3->width + cloud4->width + cloud5->width + cloud6->width + cloud7->width;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  int i,j;
  for (i = 0; i < cloud1->width; ++i)
  {
    cloud.points[i].x = cloud1->points[i].x;
    cloud.points[i].y = cloud1->points[i].y;
    cloud.points[i].z = cloud1->points[i].z;
  }
  
  j = cloud1->width; 
  for (i = 0; i<cloud2->width; ++i)
  {
    cloud.points[i+j].x = cloud2->points[i].x;
    cloud.points[i+j].y = cloud2->points[i].y;
    cloud.points[i+j].z = cloud2->points[i].z;
  }
  
  j = cloud1->width + cloud2->width; 
  for (i = 0; i<cloud3->width; ++i)
  {
    cloud.points[i+j].x = cloud3->points[i].x;
    cloud.points[i+j].y = cloud3->points[i].y;
    cloud.points[i+j].z = cloud3->points[i].z;
  }

  j = cloud1->width + cloud2->width + cloud3->width; 
  for (i = 0; i<cloud4->width; ++i)
  {
    cloud.points[i+j].x = cloud4->points[i].x;
    cloud.points[i+j].y = cloud4->points[i].y;
    cloud.points[i+j].z = cloud4->points[i].z;
  }

  j = cloud1->width + cloud2->width + cloud3->width + cloud4->width; 
  for (i = 0; i<cloud5->width; ++i)
  {
    cloud.points[i+j].x = cloud5->points[i].x;
    cloud.points[i+j].y = cloud5->points[i].y;
    cloud.points[i+j].z = cloud5->points[i].z;
  }

  j = cloud1->width + cloud2->width + cloud3->width + cloud4->width + cloud5->width; 
  for (i = 0; i<cloud6->width; ++i)
  {
    cloud.points[i+j].x = cloud6->points[i].x;
    cloud.points[i+j].y = cloud6->points[i].y;
    cloud.points[i+j].z = cloud6->points[i].z;
  }

  j = cloud1->width + cloud2->width + cloud3->width + cloud4->width + cloud5->width + cloud6->width; 
  for (i = 0; i<cloud7->width; ++i)
  {
    cloud.points[i+j].x = cloud7->points[i].x;
    cloud.points[i+j].y = cloud7->points[i].y;
    cloud.points[i+j].z = cloud7->points[i].z;
  }


  pcl::io::savePCDFileASCII ("Trial11_17/merged_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to merged_pcd.pcd." << std::endl;

  return (0);
}
