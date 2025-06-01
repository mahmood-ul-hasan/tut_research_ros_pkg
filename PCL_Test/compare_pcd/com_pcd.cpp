#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

typedef pcl::PointXYZI PointT;

void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud);
void remove_nan_data(pcl::PointCloud<PointT>::Ptr filterred_data, pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &indices);
void compare_clouds(pcl::PointCloud<PointT>::Ptr cloud1, pcl::PointCloud<PointT>::Ptr cloud2);
void modify_cloud(pcl::PointCloud<PointT>::Ptr cloud1, pcl::PointCloud<PointT>::Ptr cloud2, pcl::PointCloud<PointT>::Ptr modified_cloud);

int main (int argc, char** argv)
{
  pcl::PointCloud<PointT>::Ptr original_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr merged_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr modified_cloud (new pcl::PointCloud<PointT>);
  std::vector<int> idx;

  std::string pcd_file = "original_data.pcd";
  load_pcd_file(pcd_file, original_cloud);
  remove_nan_data(filtered_cloud, original_cloud, idx);
  pcd_file = "merged_pcd.pcd";
  load_pcd_file(pcd_file, merged_cloud);
  
  //compare_clouds(filtered_cloud, merged_cloud);
  modify_cloud(filtered_cloud, merged_cloud, modified_cloud);
  
  return (0);
}

//load the pcd file and store the point cloud data to the cloud variable
void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud)
{
  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read from the pcd file\n");
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

//compare between two cloud files
void compare_clouds(pcl::PointCloud<PointT>::Ptr cloud1, pcl::PointCloud<PointT>::Ptr cloud2)
{
  long int counter = 0;  
  for (std::size_t i = 0; i < cloud1->points.size(); ++i){
    if(abs(cloud1->points[i].x - cloud2->points[i].x) >= 1.0f || abs(cloud1->points[i].y - cloud2->points[i].y >= 1.0f) || (cloud1->points[i].z - cloud2->points[i].z >= 1.0f))
    {
        std::cout << "x:" << cloud1->points[i].x
              << "  y: "    << cloud1->points[i].y
              << "  z: "    << cloud1->points[i].z 
              << "      "   <<"x: "<<cloud2->points[i].x
              << "  y: "    <<cloud2->points[i].y
              << "  z: "    <<cloud2->points[i].z<<std::endl;
    counter++;
    }
  }
  std::cout<<"Counter: "<<counter<<std::endl;
}

void modify_cloud(pcl::PointCloud<PointT>::Ptr cloud1, pcl::PointCloud<PointT>::Ptr cloud2, pcl::PointCloud<PointT>::Ptr modified_cloud)
{
  double thx = 5.00;
  double thy = 0.50;
  double thz = 5.00;
  double counter_x = 0, counter_y = 0, counter_z = 0;

  modified_cloud->width = cloud2->width;
  modified_cloud->height   = 1;
  modified_cloud->is_dense = false;
  modified_cloud->points.resize (modified_cloud->width * modified_cloud->height);

  for (std::size_t i = 0; i < cloud1->points.size(); ++i){
    modified_cloud->points[i].intensity = cloud2->points[i].intensity;
    if(cloud1->points[i].x * cloud2->points[i].x < 0.0f)
      modified_cloud->points[i].x = cloud1->points[i].x;
    else if(abs(cloud1->points[i].x - cloud2->points[i].x) > thx){
      modified_cloud->points[i].x = cloud1->points[i].x;
       counter_x++;
    }
    else
      modified_cloud->points[i].x = cloud2->points[i].x;

    if(cloud1->points[i].y * cloud2->points[i].y < 0.0f)
      modified_cloud->points[i].y = cloud1->points[i].y;
    else if(abs(cloud1->points[i].y - cloud2->points[i].y) > thy){
      modified_cloud->points[i].y = cloud2->points[i].y;
      counter_y++;	
      if((cloud1->points[i].y >= -70) && (cloud1->points[i].y <= 20))
      	modified_cloud->points[i].intensity = 100000;
    }
    else
      modified_cloud->points[i].y = cloud2->points[i].y;
      
    if(cloud1->points[i].z * cloud2->points[i].z < 0.0f)
      modified_cloud->points[i].z = cloud1->points[i].z;
    else if(abs(cloud1->points[i].z - cloud2->points[i].z) > thz){
      modified_cloud->points[i].z = cloud1->points[i].z;
      counter_z++;
    }
    else
      modified_cloud->points[i].z = cloud2->points[i].z;

  }
  
  pcl::io::savePCDFileASCII ("modified_pcd.pcd", *modified_cloud);
  std::cerr << "Saved " << modified_cloud->points.size () << " data points to modified_pcd.pcd." << std::endl;
  std::cout<<"X-Modified: "<<counter_x<<" Y-Modified: "<<counter_y<<" Z-Modified: "<<counter_z<<std::endl;
  std::cout<<"Total Modified: "<<counter_x+counter_y+counter_z<<std::endl;
}







