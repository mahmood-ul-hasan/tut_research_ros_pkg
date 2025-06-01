#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud2 {
public:
  ros::NodeHandle nh_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  std::string target_frame_;
  std::string laser_topic_;
  std::string pointcloud_topic_;

  LaserScanToPointCloud2(ros::NodeHandle nh) :
    nh_(nh),
    laser_sub_(nh_, "input_scan", 10),
    laser_notifier_(laser_sub_, listener_, "world", 10)
  {
    // Load parameters
    nh_.param<std::string>("target_frame", target_frame_, "laser");
    nh_.param<std::string>("laser_topic", laser_topic_, "input_scan");
    nh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "output_pointcloud2");

    // Initialize subscriber and publisher with loaded parameters
    laser_sub_.subscribe(nh_, laser_topic_, 10);
    laser_notifier_.connectInput(laser_sub_);
    laser_notifier_.setTargetFrame(target_frame_);
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud2::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic_, 1);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    sensor_msgs::PointCloud2 cloud;
    try {
      projector_.transformLaserScanToPointCloud(
        target_frame_, *scan_in, cloud, listener_);
    } catch (tf::TransformException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }

    // Do something with cloud.

 // Debugging: Print out information about the PointCloud2
    ROS_INFO("Received PointCloud2 data:---------------------------------");
    ROS_INFO("  Header:");
    ROS_INFO("    seq: %d", cloud.header.seq);
    ROS_INFO("    stamp: %ld.%09ld", cloud.header.stamp.sec, cloud.header.stamp.nsec);
    ROS_INFO("    frame_id: %s", cloud.header.frame_id.c_str());
    ROS_INFO("  width: %d", cloud.width);
    ROS_INFO("  height: %d", cloud.height);
    ROS_INFO("  point_step: %d", cloud.point_step);
    ROS_INFO("  row_step: %d", cloud.row_step);
    ROS_INFO("  is_bigendian: %s", cloud.is_bigendian ? "True" : "False");
    ROS_INFO("  fields:");
    for (const auto& field : cloud.fields) {
      ROS_INFO("    name: %s", field.name.c_str());
      ROS_INFO("    offset: %d", field.offset);
      ROS_INFO("    datatype: %d", field.datatype);
      ROS_INFO("    count: %d", field.count);
    }
    ROS_INFO("  Number of points: %d", cloud.width * cloud.height);

    scan_pub_.publish(cloud);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser2pointcloud2");
  ros::NodeHandle nh("~"); // Use private NodeHandle to load parameters from the launch file
  LaserScanToPointCloud2 lstopc2(nh);
  
  ros::spin();
  
  return 0;
}
