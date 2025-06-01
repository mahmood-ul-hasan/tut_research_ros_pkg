#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_line_extraction/line_extraction.h"
#include "laser_line_extraction/line.h"
#include <sensor_msgs/PointCloud.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;


namespace line_extraction
{

class LineExtractionROS
{

public:
  // Constructor / destructor
  LineExtractionROS(ros::NodeHandle&, ros::NodeHandle&);
  ~LineExtractionROS();
  // Running
  void run();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::Subscriber scan_subscriber_;
  ros::Publisher line_publisher_;
  ros::Publisher marker_publisher_;
  ros::Publisher scan_publisher_;
  ros::Publisher pointcloud_publisher_;
      ros::Publisher comb_line_cloud_publisher_;

  // Parameters
  std::string frame_id_;
  std::string scan_topic_;
  bool pub_markers_;
  // Line extraction
  LineExtraction line_extraction_;
  bool data_cached_; // true after first scan used to cache data
  sensor_msgs::LaserScan::ConstPtr cached_scan_msg_;

  // Members
  void mergeLinesSameLine(std::vector<Line>& lines, std::vector<Line>& merged_lines);
  void createPointCloudFromLine(const std::vector<Line> &lines,   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &line_clouds,   std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &line_endpoints, pcl::PointCloud<pcl::PointXYZ>::Ptr combine_cloud);
  void publish_cloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& line_clouds, const std::string& topic);
  void publish_cloud2(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& line_clouds, const std::string& topic);


  void loadParameters();
  void populateLineSegListMsg(const std::vector<Line>&, laser_line_extraction::LineSegmentList&);
  void populateMarkerMsg(const std::vector<Line>&, visualization_msgs::Marker&);
  void populateLaserScanMsg(const std::vector<Line> &lines, sensor_msgs::LaserScan &scan_msg);
  void cacheData(const sensor_msgs::LaserScan::ConstPtr&);
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);
void populatePointCloudMsg(const std::vector<Line> &lines, sensor_msgs::PointCloud2 &cloud_msg);
Eigen::Vector4f estimatePlaneCoefficients(PointCloudT::Ptr cloud);



};

} // namespace line_extraction

#endif
