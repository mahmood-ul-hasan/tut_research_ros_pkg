#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>


void visualize_planes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_common) {


// Visualize the original clouds and the common points with different colors
        pcl::visualization::PCLVisualizer viewer("Point Clouds");
        viewer.setBackgroundColor(0, 0, 0); // Set white background
        viewer.spinOnce(100, true);


        // Create viewports for each sub-window
        int v1, v2, v3;
        viewer.createViewPort(0.0, 0.0, 0.33, 1.0, v1);
        viewer.createViewPort(0.33, 0.0, 0.66, 1.0, v2);
        viewer.createViewPort(0.66, 0.0, 1.0, 1.0, v3);


        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_color(cloud1, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color(cloud2, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_common_color(cloud_common, 0, 0, 255);

        viewer.addPointCloud<pcl::PointXYZ>(cloud1, cloud1_color, "cloud1", v1);
        viewer.addPointCloud<pcl::PointXYZ>(cloud2, cloud2_color, "cloud2", v2);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_common, cloud_common_color, "cloud_common", v3);


        // Add headings to the viewports with custom font size
        viewer.addText("Auto Extracted Planes", 10, 10, 18.0, 1.0, 1.0, 1.0, "v1_heading", v1);
        viewer.addText("Manual Extracted Planes", 10, 10, 18.0, 1.0, 1.0, 1.0, "v2_heading", v2);
        viewer.addText("Common Point Clouds", 10, 10, 18.0, 1.0, 1.0, 1.0, "v3_heading", v3);
        viewer.spinOnce(100, true);

                 }



bool checkPointCloudIntersection(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                                 double distance_threshold,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_common) {
    // Create an octree for cloud2 for efficient search
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree2(0.01); // Adjust resolution
    octree2.setInputCloud(cloud2);
    octree2.addPointsFromInputCloud();

    // Initialize the common point cloud
    cloud_common.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Iterate through points in cloud1 and add them to the common point cloud if they are close to any point in cloud2
    for (const pcl::PointXYZ& point : cloud1->points) {
        std::vector<int> point_indices;
        std::vector<float> point_distances;

        if (octree2.nearestKSearch(point, 1, point_indices, point_distances) > 0) {
            if (point_distances[0] < distance_threshold) {
                cloud_common->push_back(point);
            }
        }
    }

    return !cloud_common->empty(); // Return true if common points were found
}

void processPointCloudPair(const std::string& cloud1_pcd_file,
                           const std::string& cloud2_pcd_file,
                           double distance_threshold) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_common;

    // Load point clouds from .pcd files
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud1_pcd_file, *cloud1) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>(cloud2_pcd_file, *cloud2) == -1) {
        ROS_ERROR("Could not read point clouds from .pcd files.");
        return;
    }
    
    bool has_common_points = checkPointCloudIntersection(cloud1, cloud2, distance_threshold, cloud_common);

    if (has_common_points) {
        visualize_planes(cloud1, cloud2, cloud_common);
    } else {
        ROS_INFO("No common points found between the point clouds.");
                visualize_planes(cloud1, cloud2, cloud_common);

    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudIntersection");
    ros::NodeHandle nh;
    double distance_threshold = 0.01;


  // Load the point cloud from the PCD file
  std::string build_folder = "/home/aisl2/catkin_ws/src/latest_crane/map_optimization/build_gazebo_simple_wall_env/";
  
pcl::PCLPointCloud2::Ptr cloud_input (new pcl::PCLPointCloud2);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_input_pcl (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

// Fill in the cloud data
  pcl::PCDReader reader;
  reader.read (build_folder + "original_data.pcd", *cloud_input);
// Visualization of the cloud_filtered point cloud	
  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_input, *cloud_input_pcl);

  pcl::visualization::PCLVisualizer viewer1("Planar Components Point Clouds");	
  viewer1.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_input_pcl, 255, 255, 255);
  viewer1.addPointCloud<pcl::PointXYZ>(cloud_input_pcl, single_color, "cloud_input");	

  
  std::string cloud1_pcd_file = build_folder + "ground_seg_cloud1.pcd";
  std::string cloud2_pcd_file = build_folder + "roi_ground_seg_cloud1.pcd";
    processPointCloudPair(cloud1_pcd_file, cloud2_pcd_file, distance_threshold);

  
   std::string cloud_prefix = "wall_seg_cloud";
    std::string roi_cloud_prefix = "roi_wall_seg_cloud";
    int num_clouds = 3; // Number of point clouds to process


    for (int i = 1; i <= num_clouds; ++i) {
        std::string cloud1_pcd_file = build_folder + cloud_prefix + std::to_string(i) + ".pcd";
        std::string cloud2_pcd_file = build_folder + roi_cloud_prefix + std::to_string(i) + ".pcd";
        std::cout << cloud_prefix + std::to_string(i) + ".pcd" << std::endl;
        std::cout << roi_cloud_prefix + std::to_string(i) + ".pcd" << std::endl;
        processPointCloudPair(cloud1_pcd_file, cloud2_pcd_file, distance_threshold);
    }

    viewer1.spin();




    return 0;
}
