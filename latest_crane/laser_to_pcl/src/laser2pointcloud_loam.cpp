#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud2 {

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  std::string laser_scan_topic_;
  std::string pointcloud2_topic_;
  std::string target_frame_;

  LaserScanToPointCloud2(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, laser_scan_topic_, 10),
    laser_notifier_(laser_sub_, listener_, target_frame_, 10)
  {
    // Get parameters
    n_.param("laser_scan_topic", laser_scan_topic_, std::string("/front_laser_link/scan"));
    n_.param("pointcloud2_topic", pointcloud2_topic_, std::string("/sync_scan_cloud_filtered2"));
    n_.param("target_frame", target_frame_, std::string("camera"));

    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud2::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>(pointcloud2_topic_, 1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          target_frame_, *scan_in, cloud, listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what() << "\n";
        return;
    }
    
    // Do something with cloud.

    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "laser2pointcloud2_loam");
  ros::NodeHandle n;
  LaserScanToPointCloud2 lstopc2(n);
  
  ros::spin();
  
  return 0;
}
