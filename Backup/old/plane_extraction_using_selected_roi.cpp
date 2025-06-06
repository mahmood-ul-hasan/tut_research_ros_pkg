#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h> // Include the ROS package header
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>


typedef pcl::PointXYZI PointT;

struct WallSegmentResult {
    pcl::PointCloud<PointT>::Ptr wall_seg_cloud;
    std::vector<int> wall_inliers;
};

int i = 0;
 std::string file_directory;  // Declare the file_directory variable

 

// Function to write data to 'wall_seg_n.txt' file
void writeInlierIndicesToFile(const std::vector<int>& sIndicesSorted,
                         const std::vector<int>& iIndicesSorted,
                         const pcl::ModelCoefficients& model1, const std::stringstream& fileName) {
    // Open the file
    std::ofstream fileID(fileName.str());

    // Write the data to the file
    fileID << "Sample Indices\n";
    fileID << "Indices Size:\n";
    fileID << sIndicesSorted.size() << "\n";
    fileID << "Data:\n";
    for (size_t i = 0; i < sIndicesSorted.size(); i++) {
        fileID << sIndicesSorted[i] << "\n";
    }
    fileID << "Coefficient of the plane:\n";
    fileID << model1.values[0] << " " << model1.values[1] << " " << model1.values[2] << " " << model1.values[3] << "\n";
    fileID << "Inliers Indices\n";
    fileID << "Indices Size:\n";
    fileID << iIndicesSorted.size() << "\n";
    fileID << "Data:\n";
    for (size_t i = 0; i < iIndicesSorted.size(); i++) {
        fileID << iIndicesSorted[i] << "\n";
    }

    // Close the file
    fileID.close();
}



pcl::PointCloud<PointT>::Ptr extract_roi_and_plane_segment(const pcl::PointCloud<PointT>::Ptr& input_cloud, Eigen::Vector4f min_point, Eigen::Vector4f max_point)
// WallSegmentResult extract_roi_and_plane_segment(const pcl::PointCloud<PointT>::Ptr& input_cloud, Eigen::Vector4f min_point, Eigen::Vector4f max_point)
{
  std::cout << "Number of cloud points recieved: " << input_cloud->size() << std::endl;
  std::cout << "pcl::getFieldsList(*cloud): " << pcl::getFieldsList(*input_cloud) << std::endl;
 

    // Create a CropBox filter and apply it to the cloud
    pcl::CropBox<PointT> roi_filter;
    roi_filter.setInputCloud(input_cloud);
    roi_filter.setMin(min_point);
    roi_filter.setMax(max_point);
    pcl::PointCloud<PointT>::Ptr roi_cloud(new pcl::PointCloud<PointT>);
    roi_filter.filter(*roi_cloud);

    

    pcl::SACSegmentation<PointT> seg;
    seg.setInputCloud(roi_cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.8);
    pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr modelCoefficients(new pcl::ModelCoefficients);
    seg.segment(*inlierIndices, *modelCoefficients);

    pcl::PointCloud<PointT>::Ptr wall_seg_cloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*roi_cloud, inlierIndices->indices, *wall_seg_cloud);

// Save the inlier indices in a vector
    // std::vector<int> wall_inliers(inlierIndices->indices.begin(), inlierIndices->indices.end());

  std::vector<int> wall_inliers;

   for (size_t i = 0; i < wall_seg_cloud->size(); ++i) {
        const PointT& point = wall_seg_cloud->at(i);
        int intensity = static_cast<int>(point.intensity);
        wall_inliers.push_back(intensity);
    }




    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
viewer.setBackgroundColor(0, 0, 0);

// Add the point clouds
pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(input_cloud, 255, 255, 255);
viewer.addPointCloud<PointT>(input_cloud, single_color, "original_cloud");
viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");

pcl::visualization::PointCloudColorHandlerCustom<PointT> roi_color(roi_cloud, 255, 0, 0);
viewer.addPointCloud<PointT>(roi_cloud, roi_color, "roi_cloud");
viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "roi_cloud");

pcl::visualization::PointCloudColorHandlerCustom<PointT> wall_seg_color(roi_cloud, 0, 255, 0);
viewer.addPointCloud<PointT>(wall_seg_cloud, wall_seg_color, "wall_seg_color");
viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "wall_seg_color");


// Add the coordinate system with a larger scale factor
viewer.addCoordinateSystem(70.0);

// Add text labels for X axis
viewer.addText3D("25", pcl::PointXYZ(25.5, 0, 0), 02, 1.0, 0.0, 0.0, "x_label_5");
viewer.addText3D("50", pcl::PointXYZ(50.5, 0, 0), 2, 0.0, 1.0, 0.0, "x_label_10");
viewer.addText3D("75", pcl::PointXYZ(75.5, 0, 0), 2, 0.0, 0.0, 1.0, "x_label_15");
// Repeat for other values

// Add text labels for Y axis
viewer.addText3D("25", pcl::PointXYZ(0, 25.5, 0), 2, 1.0, 0.0, 0.0, "y_label_5");
viewer.addText3D("50", pcl::PointXYZ(0, 50.5, 0), 2, 0.0, 1.0, 0.0, "y_label_10");
viewer.addText3D("75", pcl::PointXYZ(0, 75.5, 0), 2, 0.0, 0.0, 1.0, "y_label_15");
// Repeat for other values

// Add text labels for Z axis
viewer.addText3D("25", pcl::PointXYZ(0, 0, 25.5), 2, 1.0, 0.0, 0.0, "z_label_5");
viewer.addText3D("50", pcl::PointXYZ(0, 0, 50.5), 2, 0.0, 1.0, 0.0, "z_label_10");
viewer.addText3D("75", pcl::PointXYZ(0, 0, 75.5), 2, 0.0, 0.0, 1.0, "z_label_15");
// Repeat fo
// Start the visualization loop
viewer.spin();

  // WallSegmentResult result;
  //   result.wall_seg_cloud = wall_seg_cloud;
  //   result.wall_inliers = wall_inliers;

  //   return result;


  std::cout << "wall planar: " << i << " has " << inlierIndices->indices.size () << " data points." << std::endl;
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << file_directory << "wall_seg_cloud" << i << ".pcd";
  writer.write<PointT> (ss.str (), *wall_seg_cloud, false);
  std::stringstream wall_indices_file_name;
  wall_indices_file_name << file_directory  << "wall_seg" << i << ".txt";
// Function to write data to 'wall_seg_n.txt' file
   writeInlierIndicesToFile(wall_inliers, wall_inliers, *modelCoefficients, wall_indices_file_name);
 i= i +1;

return wall_seg_cloud;

    }






int main(int argc, char** argv)
{
   
 ros::init(argc, argv, "plane_extraction_using_selected_roi");
    ros::NodeHandle nh;

 // Retrieve the file_directory parameter from the ROS parameter server
  if (nh.getParam("file_directory", file_directory)) {
    ROS_INFO("file_directory: %s", file_directory.c_str());
  } else {
    ROS_ERROR("Failed to retrieve file_directory parameter from the parameter server.");
  }
  std::string pcd_file = file_directory + "original_data.pcd";
// build_folder = file_directory;


  //   // Load the point cloud from the PCD file
  //   std::string build_folder = "/home/aisl2/catkin_ws/src/latest_crane/map_optimization/build_gazebo_simple_wall_env/";
  // std::string pcd_file = build_folder + "original_data_downsampled.pcd";

    pcl::PointCloud<PointT>::Ptr input_cloud3(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(pcd_file, *input_cloud3) == -1)
    {
        PCL_ERROR("Couldn't read PCD file.\n");
        return -1;
    }

  pcl::PointCloud<PointT>::Ptr input_cloud2 (new pcl::PointCloud<PointT>), cloud_input_pcl (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);
  // Populate the point cloud and save the index value in the intensity field of cloud_filtered_index
  for (size_t i = 0; i < input_cloud3->size(); ++i) {
      PointT point = input_cloud3->at(i); // Copy the point from cloud_filtered
      point.intensity = static_cast<float>(i); // Save the index value as intensity
      input_cloud2->push_back(point); // Add the point to cloud_filtered_index
  }


    // Define rotation angle (in radians) and axis of rotation
    // double angle = 20*M_PI / 180; // 45 degrees
    double angle = 0*M_PI / 180; // 45 degrees
    Eigen::Vector3f axis(0.0, 0.0, 1.0); // Rotate around the z-axis

    // Create a transformation matrix for rotation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(angle, axis));

    // Apply the transformation to the point cloud
    pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*input_cloud2, *input_cloud, transform);
    std::string save_pcd_file; 
save_pcd_file = file_directory + "original_data1.pcd";
    pcl::io::savePCDFileASCII(save_pcd_file, *input_cloud);  // Save the ROI cloud to a new PCD file




    pcl::PointCloud<PointT>::Ptr wall_seg_cloud(new pcl::PointCloud<PointT>);


    std::cout << "plane extraction -> 1 " << std::endl;
    // Define the Region of Interest (ROI)
    Eigen::Vector4f min_point_g(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -35, 1);
    Eigen::Vector4f max_point_g(std::numeric_limits<float>::infinity(),  std::numeric_limits<float>::infinity(), -18, 1);
    wall_seg_cloud = extract_roi_and_plane_segment(input_cloud, min_point_g, max_point_g);
    

// WallSegmentResult result = extract_roi_and_plane_segment(input_cloud, min_point_g, max_point_g);
// pcl::PointCloud<PointT>::Ptr wall_seg_cloud = result.wall_seg_cloud;
// std::vector<int> wall_inliers = result.wall_inliers;




    std::cout << "plane extraction -> 2 " << std::endl;
    Eigen::Vector4f min_point1( -80, 35, -std::numeric_limits<float>::infinity(), 1);
    Eigen::Vector4f max_point1(85, 46, std::numeric_limits<float>::infinity(), 1);
    wall_seg_cloud = extract_roi_and_plane_segment(input_cloud, min_point1, max_point1);
    save_pcd_file = file_directory + "roi_wall_seg_cloud2.pcd";
    // pcl::io::savePCDFileASCII(save_pcd_file, *wall_seg_cloud);  // Save the ROI cloud to a new PCD file


    std::cout << "plane extraction -> 3 " << std::endl;
    Eigen::Vector4f min_point2(-std::numeric_limits<float>::infinity(), 116, -std::numeric_limits<float>::infinity(), 1);
    Eigen::Vector4f max_point2(std::numeric_limits<float>::infinity(), 127, std::numeric_limits<float>::infinity(), 1);
    wall_seg_cloud = extract_roi_and_plane_segment(input_cloud, min_point2, max_point2);
    save_pcd_file = file_directory + "roi_wall_seg_cloud3.pcd";
    // pcl::io::savePCDFileASCII(save_pcd_file, *wall_seg_cloud);  // Save the ROI cloud to a new PCD file


    std::cout << "plane extraction -> 3 " << std::endl;
    // Define the Region of Interest (ROI)
    Eigen::Vector4f min_point(-70, -20, -std::numeric_limits<float>::infinity(), 1);
    Eigen::Vector4f max_point(-52.0, 60, std::numeric_limits<float>::infinity(), 1);
    wall_seg_cloud = extract_roi_and_plane_segment(input_cloud, min_point, max_point);
    save_pcd_file = file_directory + "roi_wall_seg_cloud4.pcd";
    // pcl::io::savePCDFileASCII(save_pcd_file, *wall_seg_cloud);  // Save the ROI cloud to a new PCD file




    return 0;
}
