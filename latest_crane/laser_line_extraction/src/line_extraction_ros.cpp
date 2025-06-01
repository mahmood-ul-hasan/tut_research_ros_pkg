#include "laser_line_extraction/line_extraction_ros.h"
#include <cmath>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/point_cloud2_iterator.h>


#include <pcl/filters/voxel_grid.h>

#include "matplotlibcpp.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

namespace plt = matplotlibcpp;

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

int add_first_time = true;

 // // Thresholds for KOBELCO
    float distance_threshold = 25; // Adjust this threshold as needed
    float plane_to_plane_distance_threshold = 1.5; // Adjust this threshold as needed

    float angle_threshold = 150; // Adjust this threshold as needed
    float plane_to_plane_angle_threshold = 20; // Adjust this threshold as needed

    const int check_frequency = 100;  // Frequency to check the size of merged_planes members
    const int merge_check_frequency = 500;  // Frequency to check for merging planes
    const int size_threshold = check_frequency*2;  // Adjust the threshold size as needed

    // Thresholds for GAzebo
    // float distance_threshold = 40; // Adjust this threshold as needed
    // float plane_to_plane_distance_threshold = 4; // Adjust this threshold as needed

    // float angle_threshold = 150; // Adjust this threshold as needed
    // float plane_to_plane_angle_threshold = 3; // Adjust this threshold as needed

    // const int check_frequency = 400;  // Frequency to check the size of merged_planes members
    // const int merge_check_frequency = 500;  // Frequency to check for merging planes
    // const int size_threshold = check_frequency*3;  // Adjust the threshold size as needed

namespace line_extraction
{

// Add member variables to keep track of planes and their publishers
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes_;
std::vector<ros::Publisher> plane_publishers_;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> combined_line_clouds(7);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> combined_line_clouds_filter(7);
std::vector<pcl::ModelCoefficients::Ptr> merged_plane_coefficients;


// Vector to store the merged planes
  std::vector<PointCloudT::Ptr> merged_planes;
  int iteration_counter = 0;  // Initialize an iteration counter

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS(ros::NodeHandle& nh, ros::NodeHandle& nh_local):
  nh_(nh),
  nh_local_(nh_local),
  data_cached_(false)
{
  loadParameters();
  line_publisher_ = nh_.advertise<laser_line_extraction::LineSegmentList>("line_segments", 10);
  scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &LineExtractionROS::laserScanCallback, this);
  scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("/ld_lrs3611/scan_filtered_line", 10);
  pointcloud_publisher_ =  nh_.advertise<sensor_msgs::PointCloud2>("/ld_lrs3611/scan_filtered_line_cloud", 10);
  comb_line_cloud_publisher_ =  nh_.advertise<sensor_msgs::PointCloud2>("/comb_line_cloud", 10);




  if (pub_markers_)
  {
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("line_markers", 1);
  }
}

LineExtractionROS::~LineExtractionROS()
{
}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////

void LineExtractionROS::run()
{
    
    std::cout << "============================= RUN Start ==================================== iteration_counter " << iteration_counter << std::endl;

    // Extract the lines
    std::vector<Line> lines;
    std::vector<Line> merged_lines;
    line_extraction_.extractLines(lines);


    std::cout << "before merge current_line_clouds.size()  " << lines.size() << std::endl;

    mergeLinesSameLine(lines,  merged_lines);

    std::cout << "After merge current_line_clouds.size()  " << merged_lines.size() << std::endl;

  // Vectors to store the current line clouds and endpoints
    std::vector<PointCloudT::Ptr> current_line_clouds;
    std::vector<std::pair<PointT, PointT>> current_line_endpoints;

    // Create point clouds from the current line segments
    PointCloudT::Ptr comb_line_cloud(new PointCloudT);
   

    createPointCloudFromLine(merged_lines, current_line_clouds, current_line_endpoints, comb_line_cloud);
    // Publish the current line clouds
    std::cout << "current_line_clouds.size() " << current_line_clouds.size() <<std::endl;
    std::cout << "merged_planes.size() " << merged_planes.size() << std::endl;


   


    //////////////////////////////////////////////////////////////////////////////   

if (add_first_time && !current_line_clouds.empty()) {
    
    publish_cloud2(merged_planes, "combined_line_cloud_");
    std::cout << "------------------------------------------------ add_first_time " << std::endl;
    add_first_time = false;

    for (size_t j = 0; j < current_line_clouds.size(); ++j) {
        if (current_line_clouds[j]->size() > 0) {
            std::cout << "-- Number of points in current_line_clouds[" << j << "]: " << current_line_clouds[j]->size() << std::endl;

            if (current_line_clouds[j]) {
                if (merged_planes.size() <= j) {
                    merged_planes.resize(j + 1);
                }
                if (!merged_planes[j]) {
                    merged_planes[j] = PointCloudT::Ptr(new PointCloudT);
                }
                *merged_planes[j] = *current_line_clouds[j];
                std::cout << "-- Number of points in merged_planes [" << j << "]: " << merged_planes[j]->size() << std::endl;
            }
        }
    }
    std::cout << "merged_planes.size() " << merged_planes.size() << std::endl;
    std::cout << "------------------------------------------------ add_first_time " << std::endl;
} else {
              std::set<size_t> updated_planes;  // Track updated planes in the current iteration

    for (size_t i = 0; i < current_line_clouds.size(); ++i) {
        std::cout << "--------------------------------------------line_seg No " << i << std::endl;
        PointCloudT::Ptr current_line = current_line_clouds[i];

        // std::cout << "-- Number of points in current_line_clouds[" << i << "]: " << current_line->size() << std::endl;

        if (current_line->size() > 0) {
            bool merged = false;
            float min_avg_distance = std::numeric_limits<float>::max();
            size_t closest_plane_index = 0;

            for (size_t j = 0; j < merged_planes.size(); ++j) {

               // Print the condition result
                    // std::cout << "Checking if merged_planes " << j << " has already been updated: "
                    //           << (updated_planes.find(j) != updated_planes.end()) << std::endl;

                if (updated_planes.find(j) != updated_planes.end()) {
                //  std::cout << "Skipping merged_planes " << j << " as it has already been updated in this iteration." << std::endl;
                    continue;  // Skip if this plane has already been updated
                }

                // std::cout << "merged_plane No " << j << std::endl;
                PointCloudT::Ptr plane = merged_planes[j];
                std::cout << "-- Number of points in merged_planes [" << j << "]: " << plane->size() << std::endl;

                // Build KD-tree for the current plane
                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                kdtree.setInputCloud(plane);

                float total_distance = 0.0;

                // Perform nearest neighbor search for each point in the current_line
                for (const auto& point : current_line->points) {
                    std::vector<int> pointIdxNKNSearch(1);
                    std::vector<float> pointNKNSquaredDistance(1);
                    if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                        total_distance += pointNKNSquaredDistance[0];
                    }
                }

                   float avg_distance = total_distance / current_line->points.size();
                std::cout << "avg_distance " << avg_distance << " distance_threshold " << distance_threshold << " b/w plane index " << closest_plane_index << std::endl;

                // Compare average distance to the minimum average distance found so far
                if (avg_distance < min_avg_distance) {
                    min_avg_distance = avg_distance;
                    closest_plane_index = j;
                }
            }

            if (min_avg_distance < distance_threshold) {
                // Estimate plane normal for the merged plane
                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
                pcl::PointCloud<pcl::Normal>::Ptr plane_normals(new pcl::PointCloud<pcl::Normal>);
                ne.setInputCloud(merged_planes[closest_plane_index]);
                ne.setKSearch(50); // Adjust this parameter as needed
                ne.compute(*plane_normals);

                // Estimate plane normal for the current line segment
                pcl::PointCloud<pcl::Normal>::Ptr line_normals(new pcl::PointCloud<pcl::Normal>);
                ne.setInputCloud(current_line);
                ne.compute(*line_normals);

                if (!plane_normals->empty() && !line_normals->empty()) {
                    // Calculate the angle between the normals
                    Eigen::Vector4f plane_coefficients;
                    Eigen::Vector4f line_coefficients;
                    plane_normals->at(0).getNormalVector4fMap().normalize();
                    line_normals->at(0).getNormalVector4fMap().normalize();
                    double dot_product = plane_normals->at(0).getNormalVector4fMap().dot(line_normals->at(0).getNormalVector4fMap());
                    double angle = std::acos(dot_product);

                    // std::cout << "std::fabs(angle) " << std::fabs(angle * 180/M_PI) << " angle_threshold " << angle_threshold << std::endl;
                    // if (std::fabs(angle * 180/M_PI) < angle_threshold) {
                        // Merge the point clouds
                        *merged_planes[closest_plane_index] += *current_line;  // Merge the point clouds
                        std::cout << "Number of points in combined_line_clouds[" << i << "]: " << merged_planes[closest_plane_index]->size() << std::endl;
                        std::cout << "current_line is added to merged_planes[" << closest_plane_index << "]: " << std::endl;
                        merged = true;
                        updated_planes.insert(closest_plane_index);  // Mark this plane as updated
                    // }
                }
            }

            if (!merged && merged_planes.size() < 12) {
                PointCloudT::Ptr new_plane(new PointCloudT);
                *new_plane = *current_line;
                merged_planes.push_back(new_plane);
                std::cout << "new plane is added" << std::endl;
            }
             
            if (!merged && merged_planes.size() >= 12) {
              std::cout << "current_line is discarded" << std::endl;
            }

         // Print the contents of updated_planes
                // std::cout << "Updated planes in this iteration: =========================";
                // for (const auto& index : updated_planes) {
                //     std::cout << index << " ";
                // }
                // std::cout << std::endl;

            }


        // Increment the iteration counter
        iteration_counter++;
        
        // Check size of merged_planes members after every 20 iterations
        if (iteration_counter % check_frequency == 0) {
            std::cout << "//////////////////////////////////////////////////////////////////////" << std::endl;
            std::cout << "Checking size of merged_planes members" << std::endl;
            std::cout << "//////////////////////////////////////////////////////////////////////" << std::endl;

            for (auto it = merged_planes.begin(); it != merged_planes.end(); ) {
                if ((*it)->size() < size_threshold) {
                    std::cout << "Removing small plane with size " << (*it)->size() << std::endl;
                    it = merged_planes.erase(it);
                } else {
                    ++it;
                }
            }
        }

                // Check size of merged_planes members after every 20 iterations
        // if (iteration_counter % 10 == 0) {
        //   std::cout << "-------------------------------------------------------------------------" << std::endl;
        //     for (auto it = merged_planes.begin(); it != merged_planes.end(); ) {
        //         if ((*it)->size() < 15) {
        //             std::cout << "Removing small plane with size " << (*it)->size() << std::endl;
        //             it = merged_planes.erase(it);
        //         } else {
        //             ++it;
        //         }
        //     }
        // }



         // Check for merging planes after every 500 iterations
            if (iteration_counter % merge_check_frequency == 0) {
                std::cout << "//////////////////////////////////////////////////////////////////////" << std::endl;
                std::cout << "Checking for merging similar planes" << std::endl;
                std::cout << "//////////////////////////////////////////////////////////////////////" << std::endl;

                for (size_t m = 0; m < merged_planes.size(); ++m) {
                    for (size_t n = m + 1; n < merged_planes.size(); ++n) {
                        
                         std::cout << " m " << m << " n " << n << std::endl;
                        // Estimate plane coefficients for the first plane
                        Eigen::Vector4f coefficients1 = estimatePlaneCoefficients(merged_planes[m]);

                        // Estimate plane coefficients for the second plane
                        Eigen::Vector4f coefficients2 = estimatePlaneCoefficients(merged_planes[n]);

                        std::cout << "Coefficients1: [" << coefficients1[0] << ", " << coefficients1[1] << ", " << coefficients1[2] << ", " << coefficients1[3] << "]" << std::endl;
                        std::cout << "Coefficients2: [" << coefficients2[0] << ", " << coefficients2[1] << ", " << coefficients2[2] << ", " << coefficients2[3] << "]" << std::endl;

                        // Calculate the angle between the normals
                        Eigen::Vector3f normal1 = coefficients1.head<3>();
                        Eigen::Vector3f normal2 = coefficients2.head<3>();
                        normal1.normalize();
                        normal2.normalize();
                        double dot_product = normal1.dot(normal2);
                        double angle = std::acos(dot_product);

                         std::cout << "std::fabs(angle) " << std::fabs(angle * 180/M_PI) << " angle_threshold " << plane_to_plane_angle_threshold << std::endl;
                        if (std::fabs(angle * 180/M_PI) > plane_to_plane_angle_threshold) {
                            std::cout << "skipping at angle" << std::endl;
                            continue;
                        }

                        // Calculate the distance between the planes
                        float distance = std::fabs(coefficients1[3] - coefficients2[3]);
                        std::cout << "distance " << distance << " distance_threshold " << plane_to_plane_distance_threshold << std::endl;

                        if (distance < plane_to_plane_distance_threshold) {
                            std::cout << "----------------------------------- Merging planes " << m << " and " << n << std::endl;
                            *merged_planes[m] += *merged_planes[n];
                            merged_planes.erase(merged_planes.begin() + n);
                            --n; // Adjust index after erasing
                        }
                    }
                }

              std::cout << "///////////////////////////// END /////////////////////////////////////////" << std::endl;

            }

        }

        
    }
    std::cout << "combined_line_clouds.size() " << merged_planes.size() << std::endl;


        


/////////////////////////////////////////////////////////////////////////////////////////////////


   
    
    // publish_cloud(current_line_clouds, "current_line_clouds_");
    
    sensor_msgs::PointCloud2 cloud_msg2;
    pcl::toROSMsg(*comb_line_cloud, cloud_msg2);
    cloud_msg2.header.frame_id = "world";
    cloud_msg2.header.stamp = ros::Time::now();
    comb_line_cloud_publisher_.publish(cloud_msg2);

    // publish_cloud2(combined_line_clouds, "combined_line_cloud_");
    publish_cloud2(merged_planes, "combined_line_cloud_");
    publish_cloud2(merged_planes, "combined_line_cloud_");




///////////////////////////////////////////////////////////////////////////////////////////
  // Populate message
  // laser_line_extraction::LineSegmentList msg;
  // populateLineSegListMsg(lines, msg);
  
  // // Publish the lines
  // line_publisher_.publish(msg);

  // // Also publish markers if parameter publish_markers is set to true
  // if (pub_markers_)
  // {
  //     visualization_msgs::Marker marker_msg;
  //     populateMarkerMsg(lines, marker_msg);
  //     marker_publisher_.publish(marker_msg);
  // }

  // // Publish LaserScan message
  // sensor_msgs::LaserScan scan_msg;
  // populateLaserScanMsg(lines, scan_msg);
  // scan_publisher_.publish(scan_msg);


  // // Publish LaserScan message
  // sensor_msgs::PointCloud2 cloud_msg;
  // populatePointCloudMsg(lines, cloud_msg);
  // pointcloud_publisher_.publish(cloud_msg);





}



///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::loadParameters()
{
  
  ROS_DEBUG("*************************************");
  ROS_DEBUG("PARAMETERS:");

  // Parameters used by this node
  
  std::string frame_id, scan_topic;
  bool pub_markers;

  nh_local_.param<std::string>("frame_id", frame_id, "world");
  frame_id_ = frame_id;
  // frame_id_ =   "world";
  ROS_DEBUG("frame_id: %s", frame_id_.c_str());

  nh_local_.param<std::string>("scan_topic", scan_topic, "scan");
  scan_topic_ = scan_topic;
  ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

  nh_local_.param<bool>("publish_markers", pub_markers, true);
  pub_markers_ = pub_markers;
  ROS_DEBUG("publish_markers: %s", pub_markers ? "true" : "false");

  // Parameters used by the line extraction algorithm

  double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
         max_line_gap, min_line_length, min_range, max_range, min_split_dist, outlier_dist;
  int min_line_points;

  nh_local_.param<double>("bearing_std_dev", bearing_std_dev, 1e-3);
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  ROS_DEBUG("bearing_std_dev: %f", bearing_std_dev);

  nh_local_.param<double>("range_std_dev", range_std_dev, 0.02);
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
  ROS_DEBUG("range_std_dev: %f", range_std_dev);

  nh_local_.param<double>("least_sq_angle_thresh", least_sq_angle_thresh, 1e-4);
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
  ROS_DEBUG("least_sq_angle_thresh: %f", least_sq_angle_thresh);
  
  nh_local_.param<double>("least_sq_radius_thresh", least_sq_radius_thresh, 1e-4);
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  ROS_DEBUG("least_sq_radius_thresh: %f", least_sq_radius_thresh);

  nh_local_.param<double>("max_line_gap", max_line_gap, 0.4);
  line_extraction_.setMaxLineGap(max_line_gap);
  ROS_DEBUG("max_line_gap: %f", max_line_gap);

  nh_local_.param<double>("min_line_length", min_line_length, 0.5);
  line_extraction_.setMinLineLength(min_line_length);
  ROS_DEBUG("min_line_length: %f", min_line_length);

  nh_local_.param<double>("min_range", min_range, 0.4);
  line_extraction_.setMinRange(min_range);
  ROS_DEBUG("min_range: %f", min_range);

  nh_local_.param<double>("max_range", max_range, 10000.0);
  line_extraction_.setMaxRange(max_range);
  ROS_DEBUG("max_range: %f", max_range);

  nh_local_.param<double>("min_split_dist", min_split_dist, 0.05);
  line_extraction_.setMinSplitDist(min_split_dist);
  ROS_DEBUG("min_split_dist: %f", min_split_dist);

  nh_local_.param<double>("outlier_dist", outlier_dist, 0.05);
  line_extraction_.setOutlierDist(outlier_dist);
  ROS_DEBUG("outlier_dist: %f", outlier_dist);

  nh_local_.param<int>("min_line_points", min_line_points, 9);
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  ROS_DEBUG("min_line_points: %d", min_line_points);

  ROS_DEBUG("*************************************");
}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::populateLineSegListMsg(const std::vector<Line> &lines,
                                                laser_line_extraction::LineSegmentList &line_list_msg)
{
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    // std::cout << "cit " << cit << std::endl;
    laser_line_extraction::LineSegment line_msg;
    line_msg.angle = cit->getAngle(); 
    line_msg.radius = cit->getRadius(); 
    line_msg.covariance = cit->getCovariance(); 
    line_msg.start = cit->getStart(); 
    line_msg.end = cit->getEnd(); 
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = ros::Time::now();
}

void LineExtractionROS::populateMarkerMsg(const std::vector<Line> &lines, 
                                           visualization_msgs::Marker &marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.4;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = ros::Time::now();
}




void LineExtractionROS::populateLaserScanMsg(const std::vector<Line> &lines, sensor_msgs::LaserScan &scan_msg)
{
  // Check if cached_scan_msg_ is not null
  if (!cached_scan_msg_)
  {
    ROS_ERROR("No cached scan message available.");
    return;
  }

  // Create the filtered scan message with the same metadata as the input scan
  scan_msg.header.frame_id = frame_id_;
  scan_msg.header.stamp = ros::Time::now();
  scan_msg.angle_min = cached_scan_msg_->angle_min;
  scan_msg.angle_max = cached_scan_msg_->angle_max;
  scan_msg.angle_increment = cached_scan_msg_->angle_increment;
  scan_msg.time_increment = cached_scan_msg_->time_increment;
  scan_msg.scan_time = cached_scan_msg_->scan_time;
  scan_msg.range_min = cached_scan_msg_->range_min;
  scan_msg.range_max = cached_scan_msg_->range_max;

  int num_readings = static_cast<int>((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
  scan_msg.ranges.assign(num_readings, std::numeric_limits<float>::infinity());
  scan_msg.intensities.assign(num_readings, 0.0);

  for (const auto &line : lines)
  {
    float x_start = line.getStart()[0];
    float y_start = line.getStart()[1];
    float x_end = line.getEnd()[0];
    float y_end = line.getEnd()[1];

    for (size_t i = 0; i < cached_scan_msg_->ranges.size(); ++i)
    {
      float range = cached_scan_msg_->ranges[i];
      if (range >= cached_scan_msg_->range_min && range <= cached_scan_msg_->range_max)
      {
        float angle = cached_scan_msg_->angle_min + i * cached_scan_msg_->angle_increment;
        float x = range * cos(angle);
        float y = range * sin(angle);

        // Check if the point (x, y) is on the line segment (x_start, y_start) to (x_end, y_end)
        float t = ((x - x_start) * (x_end - x_start) + (y - y_start) * (y_end - y_start)) /
                  ((x_end - x_start) * (x_end - x_start) + (y_end - y_start) * (y_end - y_start));

        if (t >= 0.0 && t <= 1.0)
        {
          float x_proj = x_start + t * (x_end - x_start);
          float y_proj = y_start + t * (y_end - y_start);
          float distance = sqrt((x - x_proj) * (x - x_proj) + (y - y_proj) * (y - y_proj));

          if (distance < 0.05)  // Threshold to determine if the point is close to the line
          {
            scan_msg.ranges[i] = range;
            scan_msg.intensities[i] = 100.0;  // You can set this to any value you want
          }
        }
      }
    }
  }
}



// Your existing includes...

void LineExtractionROS::populatePointCloudMsg(const std::vector<Line> &lines, sensor_msgs::PointCloud2 &cloud_msg)
{
  // Check if cached_scan_msg_ is not null
  if (!cached_scan_msg_)
  {
    ROS_ERROR("No cached scan message available.");
    return;
  }

  // Set the header for PointCloud2
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.header.stamp = ros::Time::now();

  // Clear the point cloud data
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.clear();

  // Define point cloud fields and size
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32,
                                "intensity", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(cached_scan_msg_->ranges.size());

  // Initialize iterators for x, y, z, and intensity channels
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity_channel(cloud_msg, "intensity");

  for (size_t i = 0; i < cached_scan_msg_->ranges.size(); ++i)
  {
    float range = cached_scan_msg_->ranges[i];
    if (range >= cached_scan_msg_->range_min && range <= cached_scan_msg_->range_max)
    {
      float angle = cached_scan_msg_->angle_min + i * cached_scan_msg_->angle_increment;
      float x = range * cos(angle);
      float y = range * sin(angle);

      bool point_belongs_to_line = false;
      for (size_t line_index = 0; line_index < lines.size(); ++line_index)
      {
        const auto &line = lines[line_index];
        float x_start = line.getStart()[0];
        float y_start = line.getStart()[1];
        float x_end = line.getEnd()[0];
        float y_end = line.getEnd()[1];

        // Check if the point (x, y) is on the line segment (x_start, y_start) to (x_end, y_end)
        float t = ((x - x_start) * (x_end - x_start) + (y - y_start) * (y_end - y_start)) /
                  ((x_end - x_start) * (x_end - x_start) + (y_end - y_start) * (y_end - y_start));

        if (t >= 0.0 && t <= 1.0)
        {
          float x_proj = x_start + t * (x_end - x_start);
          float y_proj = y_start + t * (y_end - y_start);
          float distance = sqrt((x - x_proj) * (x - x_proj) + (y - y_proj) * (y - y_proj));

          if (distance < 0.05)  // Threshold to determine if the point is close to the line
          {
            point_belongs_to_line = true;
            *intensity_channel = static_cast<float>(line_index);
            *iter_x = x;
            *iter_y = y;
            *iter_z = 0.0; // Assuming 2D scan, z can be set to 0
            ++intensity_channel;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            break;
          }
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::cacheData(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  ROS_DEBUG("Data has been cached.");
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg); 
    data_cached_ = true;
  }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);

   // Cache the most recent scan message
  cached_scan_msg_ = scan_msg;
}



///////////////////////////////////////////////////////////////////////////////
// Additional 
///////////////////////////////////////////////////////////////////////////////


void LineExtractionROS::createPointCloudFromLine(
  const std::vector<Line> &lines,
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &line_clouds,
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &line_endpoints,
  pcl::PointCloud<pcl::PointXYZ>::Ptr combine_cloud)
{
  // Check if cached_scan_msg_ is not null
  if (!cached_scan_msg_)
  {
    ROS_ERROR("No cached scan message available.");
    return;
  }



  // Resize the vectors to match the number of lines
  line_clouds.resize(lines.size());
  line_endpoints.resize(lines.size());

  // Initialize point clouds for each line
  for (auto &cloud : line_clouds)
  {
    cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  // Store the start and end points for each line
  for (size_t line_index = 0; line_index < lines.size(); ++line_index)
  {
    const auto &line = lines[line_index];
    pcl::PointXYZ start_point, end_point;

    start_point.x = line.getStart()[0];
    start_point.y = line.getStart()[1];
    start_point.z = 0.0;

    end_point.x = line.getEnd()[0];
    end_point.y = line.getEnd()[1];
    end_point.z = 0.0;

    line_endpoints[line_index] = std::make_pair(start_point, end_point);
  }

  // Iterate over the points in the cached scan message
  for (size_t i = 0; i < cached_scan_msg_->ranges.size(); ++i)
  {
    float range = cached_scan_msg_->ranges[i];
    if (range >= cached_scan_msg_->range_min && range <= cached_scan_msg_->range_max)
    {
      float angle = cached_scan_msg_->angle_min + i * cached_scan_msg_->angle_increment;
      float x = range * cos(angle);
      float y = range * sin(angle);

      // Check each line segment
      for (size_t line_index = 0; line_index < lines.size(); ++line_index)
      {
        const auto &line = lines[line_index];
        float x_start = line.getStart()[0];
        float y_start = line.getStart()[1];
        float x_end = line.getEnd()[0];
        float y_end = line.getEnd()[1];

        // Check if the point (x, y) is on the line segment (x_start, y_start) to (x_end, y_end)
        float t = ((x - x_start) * (x_end - x_start) + (y - y_start) * (y_end - y_start)) /
                  ((x_end - x_start) * (x_end - x_start) + (y_end - y_start) * (y_end - y_start));


      // std::cout << "(t >= 0.0 && t <= 1.0) t " << t  << " lines.size() " << lines.size() << std::endl;

        if (t >= 0.0 && t <= 1.0)
        {
          float x_proj = x_start + t * (x_end - x_start);
          float y_proj = y_start + t * (y_end - y_start);
          float distance = sqrt((x - x_proj) * (x - x_proj) + (y - y_proj) * (y - y_proj));

      // std::cout << "(distance < 0.1) distance " << distance  << std::endl;

          if (distance < 0.5)  // Threshold to determine if the point is close to the line
          {
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = 0.0; // Assuming 2D scan, z can be set to 0
            line_clouds[line_index]->points.push_back(point);
            combine_cloud->points.push_back(point); // Add point to combine_cloud
            break; // No need to check other lines for this point
          }
        }
      }
    }
  }

  // Set width, height and is_dense properties for the combined cloud
  combine_cloud->width = combine_cloud->points.size();
  combine_cloud->height = 1;
  combine_cloud->is_dense = true;

  // Transform the points from the laser frame to the world frame
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform("world", cached_scan_msg_->header.frame_id, ros::Time(0), ros::Duration(5.0));
    listener.lookupTransform("world", cached_scan_msg_->header.frame_id, ros::Time(0), transform);
    
  }
  catch (tf::TransformException &ex)
  {
      ROS_ERROR("Transform error: %s", ex.what());

    return;
  }

  for (auto &cloud : line_clouds)
  {
    for (auto &point : cloud->points)
    {
      tf::Vector3 pt(point.x, point.y, point.z);
      tf::Vector3 pt_transformed = transform * pt;
      point.x = pt_transformed.x();
      point.y = pt_transformed.y();
      point.z = pt_transformed.z();
    }
  }

  for (auto &point : combine_cloud->points)
  {
    tf::Vector3 pt(point.x, point.y, point.z);
    tf::Vector3 pt_transformed = transform * pt;
    point.x = pt_transformed.x();
    point.y = pt_transformed.y();
    point.z = pt_transformed.z();
  }
}



void LineExtractionROS::publish_cloud2(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& line_clouds, const std::string& topic)
{
    static std::vector<ros::Publisher> cloud_publishers;

    // Initialize publishers if they haven't been initialized yet or if the size has changed
    if (cloud_publishers.size() != line_clouds.size())
    {
        cloud_publishers.clear();  // Clear existing publishers
        cloud_publishers.resize(line_clouds.size());

        for (size_t i = 0; i < line_clouds.size(); ++i)
        {
            std::string topic_name = topic + std::to_string(i);
            cloud_publishers[i] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 20);
        }
    }

    // Publish the line clouds
    for (size_t i = 0; i < line_clouds.size(); ++i)
    {
        if (!line_clouds[i])  // Check if the pointer is null
        {
            ROS_ERROR("Null pointer encountered in combined_line_clouds at index %zu  size  %zu", i ,  line_clouds.size());
            continue;
        }

        if (line_clouds[i]->empty())
        {
            ROS_WARN("Empty point cloud encountered in combined_line_clouds at index %zu", i);
            continue;
        }

        // ROS_INFO("Publishing cloud at index %zu with %zu points", i, line_clouds[i]->points.size());

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*line_clouds[i], cloud_msg);
        cloud_msg.header.frame_id = "world";  // Assuming frame_id_ is the laser frame
        cloud_msg.header.stamp = ros::Time::now();
    
            // Publish the transformed point cloud
        cloud_publishers[i].publish(cloud_msg);
    }
}




void LineExtractionROS::publish_cloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& line_clouds, const std::string& topic)
{
    static std::vector<ros::Publisher> cloud_publishers;

    // Initialize publishers if they haven't been initialized yet or if the size has changed
    if (cloud_publishers.size() != line_clouds.size())
    {
        cloud_publishers.clear();  // Clear existing publishers
        cloud_publishers.resize(line_clouds.size());

        for (size_t i = 0; i < line_clouds.size(); ++i)
        {
            std::string topic_name = topic + std::to_string(i);
            cloud_publishers[i] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 20);
        }
    }

    // Publish the line clouds
    for (size_t i = 0; i < line_clouds.size(); ++i)
    {
        if (!line_clouds[i])  // Check if the pointer is null
        {
            ROS_ERROR("Null pointer encountered in line cloud at index %zu  size  %zu", i ,  line_clouds.size());
            continue;
        }

        if (line_clouds[i]->empty())
        {
            ROS_WARN("Empty point cloud encountered in line cloud at index %zu", i);
            continue;
        }

        // ROS_INFO("Publishing cloud at index %zu with %zu points", i, line_clouds[i]->points.size());

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*line_clouds[i], cloud_msg);
        cloud_msg.header.frame_id = "world";  // Assuming frame_id_ is the laser frame
        cloud_msg.header.stamp = ros::Time::now();
    
            // Publish the transformed point cloud
        cloud_publishers[i].publish(cloud_msg);
    }
}



void LineExtractionROS::mergeLinesSameLine(std::vector<Line>& lines, std::vector<Line>& merged_lines)
{
  for (std::size_t i = 1; i < lines.size(); ++i)
  {
    // Convert boost::array to Eigen::Vector2d
    Eigen::Vector2d p1_start(lines[i-1].getStart()[0], lines[i-1].getStart()[1]);
    Eigen::Vector2d p1_end(lines[i-1].getEnd()[0], lines[i-1].getEnd()[1]);
    Eigen::Vector2d p2_start(lines[i].getStart()[0], lines[i].getStart()[1]);
    Eigen::Vector2d p2_end(lines[i].getEnd()[0], lines[i].getEnd()[1]);

    // Calculate direction vectors
    Eigen::Vector2d dir_p1 = p1_end - p1_start;
    Eigen::Vector2d dir_p2 = p2_end - p2_start;

    // Check if the direction vectors are almost parallel
    double cos_angle = dir_p1.dot(dir_p2) / (dir_p1.norm() * dir_p2.norm());
    bool same_line = std::abs(cos_angle) > std::cos(10 * M_PI / 180); // 10 degrees tolerance

    // Merge lines if they are on the same line and overlap
    if (same_line)
    {
      // std::cout << "merged lines: angle " << cos_angle * 180/M_PI << " start1: (" << lines[i-1].getStart()[0] << ", " << lines[i-1].getStart()[1]
      //           << ") end1: (" << lines[i-1].getEnd()[0] << ", " << lines[i-1].getEnd()[1]
      //           << ") start2: (" << lines[i].getStart()[0] << ", " << lines[i].getStart()[1]
      //           << ") end2: (" << lines[i].getEnd()[0] << ", " << lines[i].getEnd()[1] << ")" << std::endl;

      // Get L, P_1, P_2 of consecutive lines
      Eigen::Vector2d L_1(lines[i-1].getRadius(), lines[i-1].getAngle());
      Eigen::Vector2d L_2(lines[i].getRadius(), lines[i].getAngle());
      Eigen::Matrix2d P_1;
      P_1 << lines[i-1].getCovariance()[0], lines[i-1].getCovariance()[1],
             lines[i-1].getCovariance()[2], lines[i-1].getCovariance()[3];
      Eigen::Matrix2d P_2;
      P_2 << lines[i].getCovariance()[0], lines[i].getCovariance()[1],
             lines[i].getCovariance()[2], lines[i].getCovariance()[3];

      // Calculate merged parameters
      Eigen::Matrix2d P_m = (P_1.inverse() + P_2.inverse()).inverse();
      Eigen::Vector2d L_m = P_m * (P_1.inverse() * L_1 + P_2.inverse() * L_2);

      // Create merged line object
      boost::array<double, 4> cov;
      cov[0] = P_m(0, 0);
      cov[1] = P_m(0, 1);
      cov[2] = P_m(1, 0);
      cov[3] = P_m(1, 1);
      std::vector<unsigned int> indices;
      indices.reserve(lines[i-1].getIndices().size() + lines[i].getIndices().size());
      indices.insert(indices.end(), lines[i-1].getIndices().begin(), lines[i-1].getIndices().end());
      indices.insert(indices.end(), lines[i].getIndices().begin(), lines[i].getIndices().end());

      Line merged_line(L_m[1], L_m[0], cov, lines[i-1].getStart(), lines[i].getEnd(), indices);

      // Project the new endpoints
      merged_line.projectEndpoints();

      // Replace current line with merged line
      lines[i] = merged_line;
    }
    else
    {
      // If not merged, retain the current line
      // std::cout << "NOT merged lines: angles:   " << cos_angle * 180/M_PI << " start1: (" << lines[i-1].getStart()[0] << ", " << lines[i-1].getStart()[1]
      //           << ") end1: (" << lines[i-1].getEnd()[0] << ", " << lines[i-1].getEnd()[1]
      //           << ") start2: (" << lines[i].getStart()[0] << ", " << lines[i].getStart()[1]
      //           << ") end2: (" << lines[i].getEnd()[0] << ", " << lines[i].getEnd()[1] << ")" << std::endl;

      merged_lines.push_back(lines[i-1]);
    }

    // Add the last line segment
    if (i == lines.size() - 1)
    {
      merged_lines.push_back(lines[i]);
    }
  }
}



// Function to estimate plane coefficients using RANSAC
Eigen::Vector4f LineExtractionROS::estimatePlaneCoefficients(PointCloudT::Ptr cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_to_plane_distance_threshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    Eigen::Vector4f plane_coefficients(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
    return plane_coefficients;
}







} // namespace line_extraction

