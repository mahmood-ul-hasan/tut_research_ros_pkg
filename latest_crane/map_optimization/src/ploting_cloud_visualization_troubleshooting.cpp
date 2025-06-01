#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<key_index.hpp>
#include<graph_optimization.hpp>
#include "matplotlibcpp.h"

typedef pcl::PointXYZI PointT;
namespace plt = matplotlibcpp;

pcl::PointCloud<PointT>::Ptr generatePlaneFromCoefficients(const Eigen::VectorXf& coefficients, const pcl::PointCloud<PointT>::Ptr input_cloud)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    float step_size = 0.5;


    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

// finding range from point cloud
    for (const auto& point : *input_cloud) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
    }


// reducing the width of plane
double range_x = max_x - min_x;
double range_y = max_y - min_y;
double range_z = max_z - min_z;
if (range_x < range_y && range_x < range_z) {
max_x = min_x + 0.1;
} else if (range_y < range_z) {
max_y = min_y + 0.1;
} else {
max_z = min_z + 0.1;
}


//  max_y = min_y+0.1;
    std::cout<<" min_x " << min_x;
    std::cout<<" max_x " << max_x;
    std::cout<<" min_y " << min_y ;
    std::cout<<" max_y " << max_y;
    std::cout<<" min_z " << min_z ;
    std::cout<<" max_z " << max_z << std::endl;


    float A = coefficients[0];
    float B = coefficients[1];
    float C = coefficients[2];
    float D = coefficients[3];

    for (float x = min_x; x <= max_x; x += step_size) {
        for (float y = min_y; y <= max_y; y += step_size) {
            for (float z = min_z; z <= max_z; z += step_size) {
                PointT point;
                point.x = x;
                point.y = y;
                point.z = z;
                cloud->push_back(point);
            }
        }
    }

    return cloud;
}


void visualize_some_2d_scan_lines_in_3d_pointcloud( pcl::PointCloud<PointT>::Ptr cloud, key_index filterred_idx)
{
key_index current_index;
current_index = filterred_idx;
pcl::PointCloud<PointT>::Ptr extracted_frame_cloud(new pcl::PointCloud<PointT>);

pcl::visualization::PCLVisualizer viewer_frame_cloud("Planar Components Point Clouds");
viewer_frame_cloud.setBackgroundColor(0, 0, 0); // Set background color to black
pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_frame_cloud(extracted_frame_cloud, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 255, 255, 255); // Green color

// Create 10 viewports
int num_rows = 3; // Number of rows of viewports
int num_cols = 5; // Number of columns of viewports
int cycle_interval = num_rows*num_cols;
std::cout << " cycle_interval " << cycle_interval << std::endl;

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

int viewport_num = 0;
double step_decimalValue = current_index.frame_id.size() / (num_rows*num_cols); // Ensure division is done with a decimal value (double)
int step_integerValue = static_cast<int>(step_decimalValue); // Convert the decimal value to an integer using static_cast
int frame_idx;

      std::cout << " ---- current_index.frame_id.size() " << current_index.frame_id.size() << std::endl;
      std::cout << " step_decimalValue " << step_decimalValue << std::endl;
      std::cout << " step_integerValue " << step_integerValue << std::endl;
      std::cout << " step_integerValue " << step_integerValue << std::endl;
      std::cout << " frame_idx " << std::endl;

// for (int i = 0; i < current_index.frame_id.size(); i++) {
for (int i = 1; i < (num_rows*num_cols); i++) {

frame_idx = i*step_integerValue;
    extract_cloud(extracted_frame_cloud, frame_idx);

    viewport_num = viewport_num+1;
    if (viewport_num >= (num_rows*num_cols))
    viewport_num = 0;

    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(i), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(extracted_frame_cloud, single_color_frame_cloud, "extracted_frame_cloud" + std::to_string(i), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "extracted_frame_cloud" + std::to_string(i));
 

}
}


void generate_all_plane_from_Ccoefficients(Eigen::VectorXf coeffs_wall1, pcl::PointCloud<PointT>::Ptr plane_points_wall1, Eigen::VectorXf coeffs_wall2, pcl::PointCloud<PointT>::Ptr plane_points_wall2, Eigen::VectorXf coeffs_wall3, pcl::PointCloud<PointT>::Ptr plane_points_wall3,
Eigen::VectorXf coeffs_wall4, pcl::PointCloud<PointT>::Ptr plane_points_wall4, Eigen::VectorXf coeffs_wall5, pcl::PointCloud<PointT>::Ptr plane_points_wall5, Eigen::VectorXf coeffs_ground1, pcl::PointCloud<PointT>::Ptr plane_points_ground1, 
Eigen::VectorXf coeffs_ground2, pcl::PointCloud<PointT>::Ptr plane_points_ground2,Eigen::VectorXf coeffs_ground3, pcl::PointCloud<PointT>::Ptr plane_points_ground3,Eigen::VectorXf coeffs_ground4, pcl::PointCloud<PointT>::Ptr plane_points_ground4,
Eigen::VectorXf coeffs_ground5, pcl::PointCloud<PointT>::Ptr plane_points_ground5){

 pcl::visualization::PCLVisualizer viewer("generate_all_plane_from_Ccoefficients");
    viewer.setBackgroundColor(0, 0, 0); // Set background color to black


pcl::PointCloud<PointT>::Ptr plane_cloud_wall1 = generatePlaneFromCoefficients(coeffs_wall1, plane_points_wall1);
pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_plane_cloud(plane_cloud_wall1, 255, 255, 255); // Green color
viewer.addPointCloud<PointT>(plane_cloud_wall1, single_color_plane_cloud, "plane_cloud_wall1");
std::cout << "coeffs_wall1 " << coeffs_wall1[0] << " " << coeffs_wall1[1] << " " << coeffs_wall1[2] << " " << coeffs_wall1[3] <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_wall2 = generatePlaneFromCoefficients(coeffs_wall2, plane_points_wall2);
viewer.addPointCloud<PointT>(plane_cloud_wall2, single_color_plane_cloud, "plane_cloud_wall2");
std::cout << "coeffs_wall2 " << coeffs_wall2[0] << " " << coeffs_wall2[1] << " " << coeffs_wall2[2] << " " << coeffs_wall2[3]  <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_wall3 = generatePlaneFromCoefficients(coeffs_wall3, plane_points_wall3);
viewer.addPointCloud<PointT>(plane_cloud_wall3, single_color_plane_cloud, "plane_cloud_wall3");
std::cout << "coeffs_wall3 " << coeffs_wall3[0] << " " << coeffs_wall3[1] << " " << coeffs_wall3[2] << " " << coeffs_wall3[3]  <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_wall4 = generatePlaneFromCoefficients(coeffs_wall4, plane_points_wall4);
viewer.addPointCloud<PointT>(plane_cloud_wall4, single_color_plane_cloud, "plane_cloud_wall4");
std::cout << "coeffs_wall4 " << coeffs_wall4[0] << " " << coeffs_wall4[1] << " " << coeffs_wall4[2] << " " << coeffs_wall4[3]  <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_wall5 = generatePlaneFromCoefficients(coeffs_wall5, plane_points_wall5);
viewer.addPointCloud<PointT>(plane_cloud_wall5, single_color_plane_cloud, "plane_cloud_wall5");
std::cout << "coeffs_wall5 " << coeffs_wall5[0] << " " << coeffs_wall5[1] << " " << coeffs_wall5[2] << " " << coeffs_wall5[3]  <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_ground1 = generatePlaneFromCoefficients(coeffs_ground1, plane_points_ground1);
viewer.addPointCloud<PointT>(plane_cloud_ground1, single_color_plane_cloud, "plane_cloud_ground1");
std::cout << "coeffs_ground1 " << coeffs_ground1[0] << " " << coeffs_ground1[1] << " " << coeffs_ground1[2] << " " << coeffs_ground1[3] << " "  <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_ground2 = generatePlaneFromCoefficients(coeffs_ground2, plane_points_ground2);
viewer.addPointCloud<PointT>(plane_cloud_ground2, single_color_plane_cloud, "plane_cloud_ground2");
std::cout << "coeffs_ground2 " << coeffs_ground2[0] << " " << coeffs_ground2[1] << " " << coeffs_ground2[2] << " " << coeffs_ground2[3] << " " <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_ground3 = generatePlaneFromCoefficients(coeffs_ground3, plane_points_ground3);
viewer.addPointCloud<PointT>(plane_cloud_ground3, single_color_plane_cloud, "plane_cloud_ground3");
std::cout << "coeffs_ground3 " << coeffs_ground3[0] << " " << coeffs_ground3[1] << " " << coeffs_ground3[2] << " " << coeffs_ground3[3] << " "  <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_ground4 = generatePlaneFromCoefficients(coeffs_ground4, plane_points_ground4);
viewer.addPointCloud<PointT>(plane_cloud_ground4, single_color_plane_cloud, "plane_cloud_ground4");
std::cout << "coeffs_ground4 " << coeffs_ground4[0] << " " << coeffs_ground4[1] << " " << coeffs_ground4[2] << " " << coeffs_ground4[3] << " "  <<std::endl;

pcl::PointCloud<PointT>::Ptr plane_cloud_ground5 = generatePlaneFromCoefficients(coeffs_ground5, plane_points_ground5);
viewer.addPointCloud<PointT>(plane_cloud_ground5, single_color_plane_cloud, "plane_cloud_ground5");
std::cout << "coeffs_ground5 " << coeffs_ground5[0] << " " << coeffs_ground5[1] << " " << coeffs_ground5[2] << " " << coeffs_ground5[3] << " "  <<std::endl;

}



void plot_index_mapping_of_one_plane(key_index filterred_idx, key_index filterred_idx_wall1,  key_index plane_idx_wall1, std::vector<int> inlier_index_wall1, std::vector<int>  roi_index_wall1)
{plt::figure();
plt::suptitle(" plot_index_mapping_of_one_plane => Indices relationship betweeen planes and input plointcloud"); // add a title
plt::subplot(4, 2, 1); plt::plot(filterred_idx.frame_id, filterred_idx.max_index,  {{"label", "filterred_idx.max_index"}}); plt::legend();  plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" max index");
plt::subplot(4, 2, 2); plt::plot(filterred_idx.frame_id, filterred_idx.min_index,  {{"label", "filterred_idx.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(4, 2, 3); plt::plot(inlier_index_wall1,  {{"label", "inlier_index_wall1"}}); plt::legend(); plt::xlabel("number of points");  plt::ylabel("index of points"); plt::grid("true");
plt::subplot(4, 2, 4); plt::plot(roi_index_wall1,  {{"label", "roi_index_wall1"}});plt::legend();   plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");
plt::subplot(4, 2, 5); plt::plot(filterred_idx.frame_id, filterred_idx.max_index,  {{"label", "filterred_idx.max_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" max index");
                       plt::plot(plane_idx_wall1.frame_id, plane_idx_wall1.max_index,  {{"label", "plane_idx_wall1.max_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" max index");


plt::subplot(4, 2, 6); plt::plot(filterred_idx_wall1.frame_id, filterred_idx_wall1.min_index,  {{"label", "filterred_idx_wall1.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(4, 2, 7); plt::plot(plane_idx_wall1.frame_id, plane_idx_wall1.max_index,  {{"label", "plane_idx_wall1.max_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" max index");
plt::subplot(4, 2, 8); plt::plot(plane_idx_wall1.frame_id, plane_idx_wall1.min_index,  {{"label", "plane_idx_wall1.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
}


void plot_filtered_idx_of_all_planes(key_index filterred_idx, key_index filterred_idx_wall1, key_index filterred_idx_wall2, key_index filterred_idx_wall3, key_index filterred_idx_wall4, key_index filterred_idx_ground1, key_index filterred_idx_ground2, key_index filterred_idx_ground3, key_index filterred_idx_ground4)
{plt::figure();
plt::suptitle("plot_filtered_idx_of_all_planes"); // add a title
plt::subplot(5, 2, 1); plt::plot(filterred_idx.frame_id, filterred_idx.max_index,  {{"label", "filterred_idx.max_index"}}); plt::legend();  plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 2); plt::plot(filterred_idx.frame_id, filterred_idx.max_index,  {{"label", "filterred_idx.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 3); plt::plot(filterred_idx_wall1.frame_id, filterred_idx_wall1.max_index,  {{"label", "filterred_idx_wall1"}}); plt::legend(); plt::xlabel("frame id");  plt::ylabel("min index"); plt::grid("true");
plt::subplot(5, 2, 4); plt::plot(filterred_idx_ground1.frame_id, filterred_idx_ground1.max_index,  {{"label", "filterred_idx_ground1"}});plt::legend();   plt::xlabel("frame id"); plt::ylabel("min index");plt::grid("true");
plt::subplot(5, 2, 5); plt::plot(filterred_idx_wall2.frame_id, filterred_idx_wall2.max_index,  {{"label", "filterred_idx_wall2.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 6); plt::plot(filterred_idx_ground2.frame_id, filterred_idx_ground2.max_index,  {{"label", "filterred_idx_ground2.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 7); plt::plot(filterred_idx_wall3.frame_id, filterred_idx_wall3.max_index,  {{"label", "filterred_idx_wall3.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 8); plt::plot(filterred_idx_ground3.frame_id, filterred_idx_ground3.min_index,  {{"label", "filterred_idx_ground3.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 9); plt::plot(filterred_idx_wall4.frame_id, filterred_idx_wall4.min_index,  {{"label", "filterred_idx_wall4.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 10); plt::plot(filterred_idx_ground4.frame_id, filterred_idx_ground4.min_index,  {{"label", "filterred_idx_ground4.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
}

void plot_inliers_idx_of_all_planes(std::vector<int> inliers_idx_ground1, std::vector<int> inliers_idx_ground2, std::vector<int> inliers_idx_ground3, std::vector<int> inliers_idx_ground4, std::vector<int> inliers_idx_ground5, std::vector<int> inlier_index_wall1, std::vector<int> inlier_index_wall2, std::vector<int> inlier_index_wall3, std::vector<int> inlier_index_wall4, std::vector<int> inlier_index_wall5 )
{plt::figure();
plt::suptitle("plot_inliers_idx_of_all_planes"); // add a title
plt::subplot(5, 2, 1); plt::plot(inliers_idx_ground1,  {{"label", "inliers_idx_ground1"}}); plt::legend(); plt::xlabel("number of points");  plt::ylabel("index of points"); plt::grid("true");
plt::subplot(5, 2, 3); plt::plot(inliers_idx_ground2,  {{"label", "inliers_idx_ground2"}}); plt::legend(); plt::xlabel("number of points");  plt::ylabel("index of points"); plt::grid("true");
plt::subplot(5, 2, 5); plt::plot(inliers_idx_ground3,  {{"label", "inliers_idx_ground3"}}); plt::legend(); plt::xlabel("number of points");  plt::ylabel("index of points"); plt::grid("true");
plt::subplot(5, 2, 7); plt::plot(inliers_idx_ground4,  {{"label", "inliers_idx_ground4"}}); plt::legend(); plt::xlabel("number of points");  plt::ylabel("index of points"); plt::grid("true");
plt::subplot(5, 2, 9); plt::plot(inliers_idx_ground5,  {{"label", "inliers_idx_ground5"}}); plt::legend(); plt::xlabel("number of points");  plt::ylabel("index of points"); plt::grid("true");

plt::subplot(5, 2, 2); plt::plot(inlier_index_wall1,  {{"label", "inlier_index_wall1"}});plt::legend();   plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");
plt::subplot(5, 2, 4); plt::plot(inlier_index_wall2,  {{"label", "inlier_index_wall2"}});plt::legend();   plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");
plt::subplot(5, 2, 6); plt::plot(inlier_index_wall3,  {{"label", "inlier_index_wall3"}});plt::legend();   plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");
plt::subplot(5, 2, 8); plt::plot(inlier_index_wall4,  {{"label", "inlier_index_wall4"}});plt::legend();   plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");
plt::subplot(5, 2, 10); plt::plot(inlier_index_wall5,  {{"label", "inlier_index_wall5"}});plt::legend();   plt::xlabel("number of points"); plt::ylabel("index of points ");plt::grid("true");

}

void plot_plane_idx_of_all_planes(key_index filterred_idx, key_index plane_idx_wall1, key_index plane_idx_wall2, key_index plane_idx_wall3, key_index plane_idx_wall4, key_index plane_idx_ground1, key_index plane_idx_ground2, key_index plane_idx_ground3, key_index plane_idx_ground4)
{plt::figure();
plt::suptitle("plot_plane_idx_of_all_planes"); // add a title
plt::subplot(5, 2, 1); plt::plot(filterred_idx.frame_id, filterred_idx.max_index,  {{"label", "filterred_idx.max_index"}}); plt::legend();  plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 2); plt::plot(filterred_idx.frame_id, filterred_idx.max_index,  {{"label", "filterred_idx.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 3); plt::plot(plane_idx_wall1.frame_id, plane_idx_wall1.max_index,  {{"label", "plane_idx_wall1"}}); plt::legend(); plt::xlabel("frame id");  plt::ylabel("min index"); plt::grid("true");
plt::subplot(5, 2, 4); plt::plot(plane_idx_ground1.frame_id, plane_idx_ground1.max_index,  {{"label", "plane_idx_ground1"}});plt::legend();   plt::xlabel("frame id"); plt::ylabel("min index");plt::grid("true");
plt::subplot(5, 2, 5); plt::plot(plane_idx_wall2.frame_id, plane_idx_wall2.max_index,  {{"label", "plane_idx_wall2.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 6); plt::plot(plane_idx_ground2.frame_id, plane_idx_ground2.max_index,  {{"label", "plane_idx_ground2.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 7); plt::plot(plane_idx_wall3.frame_id, plane_idx_wall3.max_index,  {{"label", "plane_idx_wall3.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 8); plt::plot(plane_idx_ground3.frame_id, plane_idx_ground3.min_index,  {{"label", "plane_idx_ground3.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 9); plt::plot(plane_idx_wall4.frame_id, plane_idx_wall4.min_index,  {{"label", "plane_idx_wall4.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
plt::subplot(5, 2, 10); plt::plot(plane_idx_ground4.frame_id, plane_idx_ground4.min_index,  {{"label", "plane_idx_ground4.min_index"}}); plt::legend(); plt::grid("true"); plt::xlabel("frame id"); plt::ylabel(" min index");
}



void verify_visualize_one_plane_frame_idx(pcl::PointCloud<PointT>::Ptr cloud, key_index plane_idx_ground1, std::vector<int> inliers_idx_ground1)
{

pcl::PointCloud<PointT>::Ptr combined_extracted_wall_plane1(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr combined_extracted_wall_cloud1(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr extracted_frame_cloud(new pcl::PointCloud<PointT>);


pcl::copyPointCloud(*cloud, inliers_idx_ground1, *combined_extracted_wall_plane1);

  for (int i = 1; i < plane_idx_ground1.frame_id.size(); i++)
  {
    if ((plane_idx_ground1.min_index[i] != -1) && (plane_idx_ground1.max_index[i] != -1))
    {
    extract_cloud(extracted_frame_cloud, plane_idx_ground1.frame_id[i]);
    *combined_extracted_wall_cloud1 += *extracted_frame_cloud;
    }
  }

pcl::visualization::PCLVisualizer viewer_frame_cloud("frame cloud single");
viewer_frame_cloud.setBackgroundColor(0, 0, 0); // Set background color to black
int view1, view2;
viewer_frame_cloud.createViewPort(0.0, 0.0, 0.5, 1.0, view1);  // Left half
viewer_frame_cloud.createViewPort(0.5, 0.0, 1.0, 1.0, view2);  // Right half

pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_plane1(combined_extracted_wall_plane1, 0, 255, 0); // Green color
pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_cloud1(combined_extracted_wall_cloud1, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 255, 255, 255); // Green color

int viewport_num = 1; 
viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), view1);
viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_plane1, single_color_wall_plane1, "combined_extracted_wall_plane1" + std::to_string(viewport_num), view1);
viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_plane1" + std::to_string(viewport_num));

viewport_num = 2; 
viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), view2);
viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_cloud1, single_color_wall_cloud1, "combined_extracted_wall_cloud1" + std::to_string(viewport_num), view2);
viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_cloud1" + std::to_string(viewport_num));


}



void verify_visualize_plane_frame_idx(pcl::PointCloud<PointT>::Ptr cloud, int number_of_wall_planes, key_index plane_idx_wall1, key_index plane_idx_wall2, key_index plane_idx_wall3, key_index plane_idx_wall4, key_index plane_idx_wall5, std::vector<int> inlier_index_wall1, std::vector<int> inlier_index_wall2,std::vector<int> inlier_index_wall3,std::vector<int> inlier_index_wall4,std::vector<int> inlier_index_wall5)
{
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_plane1(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_plane2(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_plane3(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_plane4(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_plane5(new pcl::PointCloud<PointT>);


  pcl::PointCloud<PointT>::Ptr extracted_frame_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_cloud1(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_cloud2(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_cloud3(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_cloud4(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr combined_extracted_wall_cloud5(new pcl::PointCloud<PointT>);


  pcl::copyPointCloud(*cloud, inlier_index_wall1, *combined_extracted_wall_plane1);
  pcl::copyPointCloud(*cloud, inlier_index_wall2, *combined_extracted_wall_plane2);
  pcl::copyPointCloud(*cloud, inlier_index_wall3, *combined_extracted_wall_plane3);
  pcl::copyPointCloud(*cloud, inlier_index_wall4, *combined_extracted_wall_plane4);
  pcl::copyPointCloud(*cloud, inlier_index_wall5, *combined_extracted_wall_plane5);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 255, 255, 255); // Green color



pcl::visualization::PCLVisualizer viewer_frame_cloud("frame cloud");
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



    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_plane1(combined_extracted_wall_plane1, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_plane2(combined_extracted_wall_plane2, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_plane3(combined_extracted_wall_plane3, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_plane4(combined_extracted_wall_plane4, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_plane5(combined_extracted_wall_plane5, 0, 255, 0); // Green color


    int viewport_num = 1; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_plane1, single_color_wall_plane1, "combined_extracted_wall_plane1" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_plane1" + std::to_string(viewport_num) );
    viewport_num = 2; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_plane2, single_color_wall_plane2, "combined_extracted_wall_plane2" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_plane2" + std::to_string(viewport_num) );
    viewport_num = 3; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_plane3, single_color_wall_plane3, "combined_extracted_wall_plane3" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_plane3" + std::to_string(viewport_num) );
    viewport_num = 4; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_plane4, single_color_wall_plane4, "combined_extracted_wall_plane4" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_plane4" + std::to_string(viewport_num) );
    viewport_num = 5; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_plane5, single_color_wall_plane5, "combined_extracted_wall_plane5" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_plane5" + std::to_string(viewport_num) );




  for (int i = 1; i < plane_idx_wall1.frame_id.size(); i++)
  {
    if (number_of_wall_planes >= 1 && (plane_idx_wall1.min_index[i] != -1) && (plane_idx_wall1.max_index[i] != -1))
    {
    extract_cloud(extracted_frame_cloud, plane_idx_wall1.frame_id[i]);
    *combined_extracted_wall_cloud1 += *extracted_frame_cloud;
    }
    
       if (number_of_wall_planes >= 2 && (plane_idx_wall2.min_index[i] != -1) && (plane_idx_wall2.max_index[i] != -1))
    {
    extract_cloud(extracted_frame_cloud, plane_idx_wall2.frame_id[i]);
    *combined_extracted_wall_cloud2 += *extracted_frame_cloud;
    }

       if (number_of_wall_planes >= 3 && (plane_idx_wall3.min_index[i] != -1) && (plane_idx_wall3.max_index[i] != -1))
    {
    extract_cloud(extracted_frame_cloud, plane_idx_wall3.frame_id[i]);
    *combined_extracted_wall_cloud3 += *extracted_frame_cloud;
    }

       if (number_of_wall_planes >= 4 && (plane_idx_wall4.min_index[i] != -1) && (plane_idx_wall4.max_index[i] != -1))
    {
    extract_cloud(extracted_frame_cloud, plane_idx_wall4.frame_id[i]);
    *combined_extracted_wall_cloud4 += *extracted_frame_cloud;
    }

       if (number_of_wall_planes >= 5 && (plane_idx_wall5.min_index[i] != -1) && (plane_idx_wall5.max_index[i] != -1))
    {
    extract_cloud(extracted_frame_cloud, plane_idx_wall5.frame_id[i]);
    *combined_extracted_wall_cloud5 += *extracted_frame_cloud;
    }
    }


    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_cloud1(combined_extracted_wall_cloud1, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_cloud2(combined_extracted_wall_cloud2, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_cloud3(combined_extracted_wall_cloud3, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_cloud4(combined_extracted_wall_cloud4, 0, 255, 0); // Green color
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_wall_cloud5(combined_extracted_wall_cloud5, 0, 255, 0); // Green color

    viewport_num = 6; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_cloud1, single_color_wall_cloud1, "combined_extracted_wall_cloud1" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_cloud1" + std::to_string(viewport_num) );

    viewport_num = 7; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_cloud2, single_color_wall_cloud2, "combined_extracted_wall_cloud2" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_cloud2" + std::to_string(viewport_num) );
    viewport_num = 8; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_cloud3, single_color_wall_cloud3, "combined_extracted_wall_cloud3" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_cloud3" + std::to_string(viewport_num) );
    viewport_num = 9; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_cloud4, single_color_wall_cloud4, "combined_extracted_wall_cloud4" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_cloud4" + std::to_string(viewport_num) );
    viewport_num = 10; 
    viewer_frame_cloud.addPointCloud<PointT>(cloud, single_color, "extracted_frame_cloud_full" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.addPointCloud<PointT>(combined_extracted_wall_cloud5, single_color_wall_cloud5, "combined_extracted_wall_cloud5" + std::to_string(viewport_num), viewport_num);
    viewer_frame_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_cloud5" + std::to_string(viewport_num) );

}
      