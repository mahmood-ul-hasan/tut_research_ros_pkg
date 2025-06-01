#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>   // Include for std::this_thread
#include <chrono>   // Include for std::chrono::milliseconds
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main() {
    // Define file path
    std::string file_path = "/media/aisl2/aisl_data/catkin_ws/src/latest_crane/laser_to_pcl/src/PCD/capture00001.pcd";

    // Load point cloud from file
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read file \n");
        return -1;
    }
    // Print field names



 std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
 for (size_t i = 0; i < std::min(cloud->size(), size_t(10)); ++i) {
        const auto& point = (*cloud)[i];    std::cout << "    " << point.x
              << " "    << point.y
              << " "    << point.z 
              << " "    << point.intensity << std::endl;
 }

 // Visualization setup
    pcl::visualization::PCLVisualizer viewer1("Point Cloud Viewer");
    viewer1.setBackgroundColor(0, 0, 0); // Set background to black

    // Add the point cloud to the viewer
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(cloud, "intensity");
    viewer1.addPointCloud<pcl::PointXYZI>(cloud, color_handler, "original_cloud");
    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");


    // Create PCLVisualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCD Viewer"));
    viewer->initCameraParameters();

    // Define colors for each intensity category (1-4)
    std::vector<std::tuple<float, float, float>> colors = {
        std::make_tuple(1.0f, 0.0f, 0.0f),   // Red for intensity 1
        std::make_tuple(0.0f, 1.0f, 0.0f),   // Green for intensity 2
        std::make_tuple(0.0f, 0.0f, 1.0f),   // Blue for intensity 3
        std::make_tuple(1.0f, 1.0f, 1.0f)    // White for intensity 4
    };

    // Create separate point clouds for each intensity category
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> intensity_clouds(4); // 4 categories (intensity 1-4)

    // Iterate through points and categorize based on intensity
    for (const auto& point : cloud->points) {
        int intensity = static_cast<int>(point.intensity);
        if (intensity >= 1 && intensity <= 4) {
            if (!intensity_clouds[intensity - 1]) {
                intensity_clouds[intensity - 1].reset(new pcl::PointCloud<pcl::PointXYZI>);
            }
            intensity_clouds[intensity - 1]->points.push_back(point);
        }
    }

    // Visualize each intensity category with a specific color
    int color_index = 0;
    for (const auto& intensity_cloud : intensity_clouds) {
        if (intensity_cloud) {
            // Add cloud to viewer with a specific color
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_handler(intensity_cloud,
                                                                                           std::get<0>(colors[color_index]),
                                                                                           std::get<1>(colors[color_index]),
                                                                                           std::get<2>(colors[color_index]));
            viewer->addPointCloud(intensity_cloud, color_handler, "cloud_intensity_" + std::to_string(color_index + 1));
            color_index++;
        }
    }

    // Spin viewer
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
