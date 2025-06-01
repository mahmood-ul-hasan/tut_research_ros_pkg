#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/point_cloud.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <ros/package.h> 


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


pcl::PointCloud<pcl::PointXYZ>::Ptr extractLargestCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, std::vector<int> &largest_cluster_indices, float cluster_tolerance)
{
  
    // Create the KdTree object
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_cloud);

    // Euclidean Cluster Extraction parameters
    int min_cluster_size = 100;     // Minimum number of points that a cluster should have
    int max_cluster_size = 2500000;   // Maximum number of points that a cluster should have

    // Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);


    // std::cout << "Input cloud size: " << input_cloud->size();
    // std::cout << " --- Number of cluster indices found: " << cluster_indices.size() << std::endl;
    // Find the largest cluster
    int largest_cluster_index = 0;
    size_t largest_cluster_size = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        if (cluster_indices[i].indices.size() > largest_cluster_size)
        {
            largest_cluster_size = cluster_indices[i].indices.size();
            largest_cluster_index = i;
        }
    }

    // Update the largest cluster indices in the argument
    largest_cluster_indices = cluster_indices[largest_cluster_index].indices;


    // Extract the largest cluster from the input point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &index : largest_cluster_indices)
    {
        largest_cluster->points.push_back(input_cloud->points[index]);
    }
    largest_cluster->width = static_cast<uint32_t>(largest_cluster->points.size());
    largest_cluster->height = 1;
    largest_cluster->is_dense = true;

    return largest_cluster;
}



int main (int argc, char** argv)
{
 
   ros::init(argc, argv, "plane_extraction");
  ros::NodeHandle nh;

  std::string file_directory;  // Declare the file_directory variable


  double ground_angle_degree;
  double ground_distance_threshold;
  double min_ground_plane_size;
  double ground_plane_cluster_Tolerance;
  double wall_distance_threshold;
  double wall_angle_degree;
  double min_wall_plane_size;
  double wall_plane_cluster_Tolerance;
  int min_wall_segment_cluster_size;
  int max_wall_segment_cluster_size;
  double wall_segment_cluster_Tolerance;
  double downsampling_leaf_size;




  // // Retrieve parameters from the ROS parameter server
  nh.param<double>("ground_angle_degree", ground_angle_degree, 0.0);
  nh.param<double>("ground_distance_threshold", ground_distance_threshold, 0.0);
  nh.param<double>("min_ground_plane_size", min_ground_plane_size, 0.0);
  nh.param<double>("ground_plane_cluster_Tolerance", ground_plane_cluster_Tolerance, 0.0);  // Check parameter name
  nh.param<double>("wall_distance_threshold", wall_distance_threshold, 0.0);
  nh.param<double>("wall_angle_degree", wall_angle_degree, 0.0);
  nh.param<double>("min_wall_plane_size", min_wall_plane_size, 0.0);
  nh.param<double>("wall_plane_cluster_Tolerance", wall_plane_cluster_Tolerance, 0.0);  // Check parameter name
  nh.param<int>("min_wall_segment_cluster_size", min_wall_segment_cluster_size, 0);
  nh.param<int>("max_wall_segment_cluster_size", max_wall_segment_cluster_size, 0);
  nh.param<double>("wall_segment_cluster_Tolerance", wall_segment_cluster_Tolerance, 0.0);
  nh.param<double>("downsampling_leaf_size", downsampling_leaf_size, 0.05);

ROS_INFO("Checking if the parameter exists: %d", nh.hasParam("file_directory"));


 // Retrieve the file_directory parameter from the ROS parameter server
  if (nh.getParam("file_directory", file_directory)) {
    ROS_INFO("file_directory: %s", file_directory.c_str());
  } else {
    ROS_ERROR("Failed to retrieve file_directory parameter from the parameter server.");
  }


  pcl::PCLPointCloud2::Ptr cloud_input (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_input_pcl (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the input cloud data
  pcl::PCDReader reader;
  reader.read (file_directory + "original_data.pcd", *cloud_input);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_input, *cloud_filtered);

  // downsampling 
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setLeafSize(downsampling_leaf_size, downsampling_leaf_size, downsampling_leaf_size); // Set the voxel grid size along x, y, and z axes
  sor.filter(*cloud_filtered);

  // save after downsampling
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << file_directory << "original_data_downsampled.pcd";
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  Eigen::Vector3f ground_axis;
  ground_axis << 0.0, 0.0, 1.0; // Ground plane; plane perpendicular to the Z-axis
  float ground_angle = ground_angle_degree * M_PI / 180.0;; // Set the desired angle in degrees (e.g., 40.0)

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setEpsAngle(ground_angle);
  seg.setDistanceThreshold (ground_distance_threshold);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  int i = 1, nr_points = (int) cloud_filtered->size ();


  // Visualization of the cloud_filtered point cloud	
  pcl::visualization::PCLVisualizer viewer("Planar Components Point Clouds");	
  viewer.setBackgroundColor(0, 0, 0);

   // Create viewports for each sub-window
    int v1, v2, v3, v4, v5, v6;
    viewer.createViewPort(0.0, 0.5, 0.333, 1.0, v1);  // Left-top
    viewer.createViewPort(0.333, 0.5, 0.667, 1.0, v2); // Center-top
    viewer.createViewPort(0.667, 0.5, 1.0, 1.0, v3);   // Right-top
    viewer.createViewPort(0.0, 0.0, 0.333, 0.5, v4);   // Left-bottom
    viewer.createViewPort(0.333, 0.0, 0.667, 0.5, v5); // Center-bottom
    viewer.createViewPort(0.667, 0.0, 1.0, 0.5, v6);   // Right-bottom

    // Add headings at the center of each viewport
    // viewer.addText("Input Point Cloud", 250, 450, 15, 1.0, 1.0, 1.0, "v1_heading", v1);
    // viewer.addText("Point Cloud after Ground Extraction", 250, 450, 15, 1.0, 1.0, 1.0, "v2_heading", v2);
    // viewer.addText("Clustered Point Cloud", 250, 450, 15, 1.0, 1.0, 1.0, "v3_heading", v3);
    // viewer.addText("Combine Extracted planes and input", 250, 450, 15, 1.0, 1.0, 1.0, "v4_heading", v4);
    // viewer.addText("Extracted Ground Planes", 250, 450, 15, 1.0, 1.0, 1.0, "v5_heading", v5);
    // viewer.addText("Extracted Wall Planes", 250, 450, 15, 1.0, 1.0, 1.0, "v6_heading", v6);


  // List of different colors
  std::vector<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> colors;
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_g, 255, 0, 0));      // Red
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_g, 0, 255, 0));      // Green
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_g, 0, 128, 0));       // Dark Green
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_g, 255, 0, 128));     // Pink
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_g, 255, 0, 255));// 2. Magenta
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_g, 154, 205, 50));// 3. Yellow-Green
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_g, 255, 215, 0)); // 4. Gold
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_g, 255, 255, 255);

  viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color, "input_cloud_filtered", v4);	
  viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color, "input_cloud_filtered_all", v1);

  pcl::visualization::PCLVisualizer viewer_ground_plane("Seprate ground Planar Components");	
  viewer_ground_plane.setBackgroundColor(0, 0, 0);
 // Create viewports for each sub-window
    int v1_ground, v2_ground, v3_ground, v4_ground, v5_ground, v6_ground;
    viewer_ground_plane.createViewPort(0.0, 0.5, 0.333, 1.0, v1_ground);  // Left-top
    viewer_ground_plane.createViewPort(0.333, 0.5, 0.667, 1.0, v2_ground); // Center-top
    viewer_ground_plane.createViewPort(0.667, 0.5, 1.0, 1.0, v3_ground);   // Right-top
    viewer_ground_plane.createViewPort(0.0, 0.0, 0.333, 0.5, v4_ground);   // Left-bottom
    viewer_ground_plane.createViewPort(0.333, 0.0, 0.667, 0.5, v5_ground); // Center-bottom
    viewer_ground_plane.createViewPort(0.667, 0.0, 1.0, 0.5, v6_ground);   // Right-bottom

    // Add headings at the center of each viewport
    viewer_ground_plane.addText("All Ground Plane", 250, 450, 15, 1.0, 1.0, 1.0, "v1_heading", v1_ground);
    viewer_ground_plane.addText("Largest Clustered Plane", 250, 450, 15, 1.0, 1.0, 1.0, "v2_heading", v2_ground);
    viewer_ground_plane.addText("Ground Plane 01", 250, 450, 15, 1.0, 1.0, 1.0, "v3_heading", v3_ground);
    viewer_ground_plane.addText("Ground Plane 02", 250, 450, 15, 1.0, 1.0, 1.0, "v4_heading", v4_ground);
    viewer_ground_plane.addText("Ground Plane 03", 250, 450, 15, 1.0, 1.0, 1.0, "v5_heading", v5_ground);
    viewer_ground_plane.addText("Ground Plane 04", 250, 450, 15, 1.0, 1.0, 1.0, "v6_heading", v6_ground);




  // While 10% of the original cloud is still there  to extract the ground plane
  while (cloud_filtered->size () > 0.1 * nr_points)
  {
    
    // Segment the ground planar component from the remaining cloud
    seg.setAxis(ground_axis);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    // Convert pcl::PointIndices::Ptr to std::vector<int>
    std::vector<int> ground_inliers = inliers->indices;


    if (inliers->indices.size () < min_ground_plane_size)
    {
      std::cerr << "Estimated ground plane size is less than 'min_ground_plane_size' so stopping planes extraction" << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_g);


    std::vector<int> ground_largest_cluster_indices;
 bool ground_plane_cluster_extraction;
 ground_plane_cluster_extraction = false;

 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g_cluster;
 if (ground_plane_cluster_extraction == true){
    // extract Largest Cluster
  cloud_g_cluster = extractLargestCluster(cloud_g, ground_largest_cluster_indices, ground_plane_cluster_Tolerance);

  }
  else if(ground_plane_cluster_extraction == false){
    cloud_g_cluster = cloud_g;
  }
    // Save the extracted Cluster
    std::cout << "Ground planar: " << i << " has " << inliers->indices.size ()  << " data points." << std::endl;
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << file_directory << "ground_seg_cloud" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_g_cluster, false);
    std::stringstream ground_indices_file_name;
    ground_indices_file_name << file_directory <<  "ground_seg" << i << ".txt";
    // Function to write data to 'wall_seg_n.txt' file

     if (ground_plane_cluster_extraction == true){
    writeInlierIndicesToFile(ground_inliers, ground_largest_cluster_indices, *coefficients, ground_indices_file_name);
     }
       else if(ground_plane_cluster_extraction == false){
    writeInlierIndicesToFile(ground_inliers, ground_inliers, *coefficients, ground_indices_file_name);
       }


    // Visualization of the cloud_g point cloud	
    viewer.addPointCloud<pcl::PointXYZ>(cloud_g_cluster, colors[i-1 % colors.size()], ss.str(), v4);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_g_cluster, colors[i-1 % colors.size()], ss.str()+ "_all", v5);

    viewer_ground_plane.addPointCloud<pcl::PointXYZ>(cloud_g, colors[i-1 % colors.size()], ss.str()+ "_all", v1_ground);
    viewer_ground_plane.addPointCloud<pcl::PointXYZ>(cloud_g_cluster, colors[i-1 % colors.size()], ss.str()+ "_all_clustered", v2_ground);
    viewer_ground_plane.addPointCloud<pcl::PointXYZ>(cloud_g_cluster, colors[i-1 % colors.size()], ss.str(), v2_ground + i);
    viewer_ground_plane.spinOnce(100, true);


    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;


  }

    std::cout << "extracting ground plane finished " << i << std::endl;
    std::cout << "==================================================================================" << std::endl;


 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_red(cloud_filtered, 255, 255, 255); // Red color
    // pcl::visualization::PCLVisualizer viewer_after("Red Point Cloud");
    // viewer_after.setBackgroundColor(0, 0, 0);
    // viewer_after.addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color_red, "red_point_cloud");
    // viewer_after.spinOnce(100, true);



      viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color_red, "after_ground_extraction",v2);	
    viewer.spinOnce(100, true);


pcl::visualization::PCLVisualizer viewer_wall_all("All Extracted Wall Planes");	
viewer_wall_all.setBackgroundColor(0, 0, 0);
    int v1_wall, v2_wall, v3_wall, v4_wall, v5_wall, v6_wall;
    viewer_wall_all.createViewPort(0.0, 0.5, 0.333, 1.0, v1_wall);  // Left-top
    viewer_wall_all.createViewPort(0.333, 0.5, 0.667, 1.0, v2_wall); // Center-top
    viewer_wall_all.createViewPort(0.667, 0.5, 1.0, 1.0, v3_wall);   // Right-top
    viewer_wall_all.createViewPort(0.0, 0.0, 0.333, 0.5, v4_wall);   // Left-bottom
    viewer_wall_all.createViewPort(0.333, 0.0, 0.667, 0.5, v5_wall); // Center-bottom
    viewer_wall_all.createViewPort(0.667, 0.0, 1.0, 0.5, v6_wall);   // Right-bottom

    // Add headings at the center of each viewport
    viewer_wall_all.addText("All Wall Plane", 250, 450, 15, 1.0, 1.0, 1.0, "v1_heading", v1_wall);
    viewer_wall_all.addText("Largest Clustered Wall Plane", 250, 450, 15, 1.0, 1.0, 1.0, "v2_heading", v2_wall);
    // viewer_wall_all.addText("Ground Plane 02", 250, 450, 15, 1.0, 1.0, 1.0, "v3_heading", v3_wall);
    // viewer_wall_all.addText("Ground Plane 03", 250, 450, 15, 1.0, 1.0, 1.0, "v4_heading", v4_wall);
    // viewer_wall_all.addText("Ground Plane 04", 250, 450, 15, 1.0, 1.0, 1.0, "v5_heading", v5_wall);
    // viewer_wall_all.addText("Ground Plane 05", 250, 450, 15, 1.0, 1.0, 1.0, "v6_heading", v6_wall);




float wall_angle =  wall_angle_degree * M_PI / 180.0;; // Set the desired angle in degrees (e.g., 40.0)
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_w (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::ModelCoefficients> all_coefficients;


//------------------------

pcl::visualization::PCLVisualizer viewer_frame_cloud("Extracted frame cloud ");
viewer_frame_cloud.setBackgroundColor(0, 0, 0); // Set background color to black

// Create 10 viewports
int num_rows = 2; // Number of rows of viewports
int num_cols = 5; // Number of columns of viewports
int cycle_interval = num_rows*num_cols;

for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
        int viewport_num = row * num_cols + col + 1; // Calculate the viewport number
        double x_min = static_cast<double>(col) / num_cols;
        double x_max = static_cast<double>(col + 1) / num_cols;
        double y_min = 1.0 - static_cast<double>(row + 1) / num_rows;
        double y_max = 1.0 - static_cast<double>(row) / num_rows;
        viewer_frame_cloud.createViewPort(x_min, y_min, x_max, y_max, viewport_num);
    }
}
//--------------------------



    // Create a separate point cloud for each cluster	
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);	
    std::vector<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> colors_wall;
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 0, 0, 255));      // Blue
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 255, 255, 0));    // Yellow
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 128, 0, 128));    // Purple
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 255, 165, 0));    // Orange
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 0, 128, 128)); // 5. Teal
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 0, 0, 139)); // 6. Dark Blue
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 220, 20, 60));// 7. Crimson 
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 50, 205, 50)); // 8. Lime Green
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 135, 206, 235)); // 9. Sky Blu
    colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cluster_cloud, 230, 230, 250)); // 10. Lavender



pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
tree->setInputCloud(cloud_filtered);
// Euclidean Cluster Extraction
std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
ec.setClusterTolerance(wall_segment_cluster_Tolerance);
ec.setMinClusterSize(min_wall_segment_cluster_size);
ec.setMaxClusterSize(max_wall_segment_cluster_size);
ec.setSearchMethod(tree);
ec.setInputCloud(cloud_filtered);
ec.extract(cluster_indices);

std::cout << "total cluster founded " <<  cluster_indices.size() << std::endl;
int wall_num =1;



std::vector<int> cluster_cloud_indices;
std::vector<int> wall_largest_cluster_indices;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_w_cluster ;


size_t cluster_idx = 0;
std::vector<pcl::PointIndices> cluster_indices_1;
cluster_indices_1.push_back(cluster_indices[0]);


    // Extract the points for the current cluster
    for (const auto& point_index : cluster_indices[cluster_idx].indices)
    {
        cluster_cloud->push_back((*cloud_filtered)[point_index]);
        cluster_cloud_indices.push_back(point_index); // Add the index to the vector

    }

    std::cout << "------------------- Cluster " << cluster_idx << " size: " << cluster_cloud->size() << std::endl;


    pcl::ModelCoefficients::Ptr w_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr w_inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg_wall;
    seg_wall.setOptimizeCoefficients (true);
    // seg_wall.setModelType (pcl::SACMODEL_PLANE);
    seg_wall.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg_wall.setMethodType (pcl::SAC_RANSAC);
    seg_wall.setMaxIterations (10000);
    seg_wall.setEpsAngle(wall_angle);
    seg_wall.setDistanceThreshold (wall_distance_threshold);
    int nr_wall_points = (int) cluster_cloud->size ();

    i= 1;
    Eigen::Vector3f wall_axis;
    wall_axis << 1.0, 0.0, 0.0; // Wall plane; plane perpendicular to the X-axis (assuming walls are parallel to the YZ-plane)
    std::cerr << "Segment the wall plane in the X-axis" << std::endl;

    while (cluster_cloud->size() > 0.1 * nr_wall_points)
    {
      seg_wall.setAxis(wall_axis);
      seg_wall.setInputCloud(cluster_cloud);
      seg_wall.segment(*w_inliers, *w_coefficients);

  // If the found plane in the X-axis is not large enough, check the Y-axis
    if (w_inliers->indices.size() < min_wall_plane_size) {
      std::cerr << "Segment the wall plane in the Y-axis" << std::endl;

      // Segment the wall planar component from the remaining cloud in the Y-axis
      wall_axis << 0.0, 1.0, 0.0; // Wall plane; plane perpendicular to the Y-axis (assuming walls are parallel to the XZ-plane)
      seg_wall.setAxis(wall_axis);
      seg_wall.setInputCloud(cluster_cloud);
      seg_wall.segment(*w_inliers, *w_coefficients);
  }



    std::cout << "plane " <<  i <<  " of cluster " << cluster_idx << " has size: " << w_inliers->indices.size() << std::endl;

    if (w_inliers->indices.size() < min_wall_plane_size)
    { std::cerr << "Estimated wall plane " << i << " of cluster " << cluster_idx << " is less than min_wall_plane_size" << std::endl;
    break;
    }


    // Extract the inliers
    extract.setInputCloud (cluster_cloud);
    extract.setIndices (w_inliers);
    extract.setNegative (false);
    extract.filter (*cloud_w);


    
  std::vector<int> wall_inliers = w_inliers->indices;

//====================================================
    std::cout << "plane and extracted_cluster_cloud"  << std::endl;
  std::vector<int> point_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::copyPointCloud(*cloud_filtered, cluster_indices_1, *extracted_cluster_cloud);
  pcl::copyPointCloud(*extracted_cluster_cloud, wall_inliers, *extracted_cloud_plane);


  int viewport_num;
  viewport_num =  wall_num;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_wall_cloud1(extracted_cloud_plane, 0, 255, 0); // Green color
  viewer_frame_cloud.addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
  viewer_frame_cloud.addPointCloud<pcl::PointXYZ>(extracted_cloud_plane, single_color_wall_cloud1, "extracted_frame_cloud"+ std::to_string(viewport_num), viewport_num);
  viewer_frame_cloud.addPointCloud<pcl::PointXYZ>(cloud_w, single_color_wall_cloud1, "extracted_frame_cloud_plane"+ std::to_string(viewport_num), 5+ viewport_num);
  viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "extracted_frame_cloud" + std::to_string(viewport_num));

      std::cout << "Finished plane and extracted_cluster_cloud"  << std::endl;

//=====================================================



    // Add the plane point cloud to the viewer with the assigned color
    // std::stringstream ss;
    ss << "Plane_" << cluster_idx;
    viewer.addPointCloud<pcl::PointXYZ>(cluster_cloud, colors_wall[cluster_idx % colors_wall.size()], ss.str(), v3);


//  bool wall_plane_cluster_extraction;
//  wall_plane_cluster_extraction = false;

//  if (wall_plane_cluster_extraction == true){
//     // extract Largest Cluster
//     std::cout << "wall_plane_cluster_extraction" << std::endl;
//    cloud_w_cluster = extractLargestCluster(cloud_w, wall_largest_cluster_indices, wall_plane_cluster_Tolerance);
//       }
//    else if(wall_plane_cluster_extraction == false){
//     cloud_w_cluster = cloud_w;
//    }

    cloud_w_cluster = cloud_w;

    // saving inliar
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << file_directory << "wall_seg_cloud" << wall_num << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_w_cluster, false);

              std::cout << "Finished plane and extracted_cluster_cloud 2 "  << std::endl;


    std::stringstream wall_indices_file_name;
    wall_indices_file_name << file_directory << "wall_seg" << wall_num << ".txt";
          std::cout << "Finished plane and extracted_cluster_cloud 3 "  << std::endl;

    // Function to write data to 'wall_seg_n.txt' file
    //  if (wall_plane_cluster_extraction == true){
    // writeInlierIndicesToFile(wall_inliers, wall_largest_cluster_indices, *w_coefficients, wall_indices_file_name);
    //  }
    //     else if(wall_plane_cluster_extraction == false)
        {
    writeInlierIndicesToFile(cluster_cloud_indices, wall_inliers, *w_coefficients, wall_indices_file_name);
        }

          std::cout << "Finished plane and extracted_cluster_cloud 4 "  << std::endl;

    ss << "Plane_" << cluster_idx << "_i_" << i;
    viewer.addPointCloud<pcl::PointXYZ>(cloud_w_cluster, colors_wall[cluster_idx + i % colors_wall.size()], ss.str(), v6);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_w_cluster, colors_wall[cluster_idx + i % colors_wall.size()], ss.str()+ "_all", v4);

      std::cout << "Convert the numeric variables to strings"  << std::endl;

// Convert the numeric variables to strings
    std::string i_str = std::to_string(i);
    std::string cluster_idx_str = std::to_string(cluster_idx);
    // Concatenate the strings to form the title
    std::string title = "Wall Plane " + std::to_string(wall_num) + " (plane " + i_str + " of cluster " + cluster_idx_str + ") ";

    viewer_wall_all.addText(title, 250, 450, 15, 1.0, 1.0, 1.0, "heading_" + std::to_string(v2_wall + wall_num), v2_wall + wall_num);


    viewer_wall_all.addPointCloud<pcl::PointXYZ>(cloud_w, colors_wall[cluster_idx + i % colors_wall.size()], ss.str()+ "no_cluster_all", v1_wall );
    viewer_wall_all.addPointCloud<pcl::PointXYZ>(cloud_w_cluster, colors_wall[cluster_idx + i % colors_wall.size()], ss.str()+ "_all", v2_wall);
    viewer_wall_all.addPointCloud<pcl::PointXYZ>(cloud_w_cluster, colors_wall[cluster_idx + i % colors_wall.size()], ss.str(), v2_wall + wall_num);


    viewer_wall_all.spinOnce(100, true);



    // Remove the inliers of the plane from the remaining cloud
    extract.setNegative(true);
    extract.filter (*cloud_f);
    cluster_cloud.swap (cloud_f);
    i++;
wall_num = wall_num +1;

    }





viewer.spin();

return (0);
}
