#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <queue>

#include "laser_assembler/base_assembler.h"
#include "filters/filter_chain.h"

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// #include <pcl_ros/point_cloud.h>

// #include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <matplotlibcpp.h>

#include <rosbag/bag.h>

#include <tf2_ros/transform_broadcaster.h>

using namespace laser_geometry;
using namespace std ;
using namespace laser_assembler ;


bool first_time_stamp_cloud_bool = true;
ros::Time first_time_stamp_cloud; // Initialize as default value
rosbag::Bag bag_;


class LidarPoseTransformer
{
public:
    LidarPoseTransformer() : filter_chain_("sensor_msgs::LaserScan")
     {
         // ***** Set Laser Projection Method *****
        nh_.param("ignore_laser_skew", ignore_laser_skew_, true);
        // configure the filter chain from the parameter server
        filter_chain_.configure("filters", nh_);


        lidar_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/ld_lrs3611/scan", 1, &LidarPoseTransformer::lidarCallback, this);
        path_sub_ = nh_.subscribe<nav_msgs::Path>("/pose_graph/pose_graph_path", 1, &LidarPoseTransformer::pathCallback, this);
        bag_.open("/media/aisl2/aisl_data/catkin_ws/src/latest_crane/laser_assembler/data.bag", rosbag::bagmode::Write);
        timer_ = nh_.createTimer(ros::Duration(60.0), &LidarPoseTransformer::periodicTask, this);


    }

    
   void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // Write the LiDAR message to the bag file
        bag_.write("/scan_history", scan_msg->header.stamp, *scan_msg);
    }


void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // Lock the mutex before accessing the shared deque
    poses_hist_mutex_.lock();

// Clear the deque to reset it and remove all previous poses
    poses_hist.clear();

    // Iterate through each pose in the path message
    for (const auto& pose : msg->poses) {
        if (poses_hist.size() == max_poses_) {
            // We're removing an element, so this reduces our total pose count
            total_poses_ -= 1 ;
            poses_hist.pop_front(); // The front of the deque has the oldest element, so we can get rid of it
        }
        poses_hist.push_back(pose); // Add the newest pose to the back of the deque
        // Update the total number of poses
        total_poses_ += 1;
    }

    // Print the current number of poses after processing all poses
    // printf("Poses ---: %4u  Points: %10u\n", poses_hist.size(), total_poses_);

    // Unlock the mutex after processing the shared deque
    poses_hist_mutex_.unlock();
    
    ROS_DEBUG("done with pathCallback");

        // Match poses and clouds based on timestamps
    // matchPosesAndClouds();
}





void periodicTask(const ros::TimerEvent& event) {
    // Check if there are scans and poses available
    if (scan_hist.empty() || poses_hist.empty()) {
        ROS_WARN("No lidar scans or poses available.");
        return;
    }

        // Make a copy of scan_hist
        std::deque<sensor_msgs::LaserScan> scan_hist_copy = scan_hist;
        std::deque<geometry_msgs::PoseStamped> poses_hist_copy = poses_hist;

   auto pose_it = poses_hist_copy.begin();
        const ros::Duration time_diff = scan_hist.front().header.stamp - poses_hist_copy.front().header.stamp;


// store in bag file
rosbag::Bag bag;
bag.open("/media/aisl2/aisl_data/catkin_ws/src/latest_crane/laser_assembler/data.bag", rosbag::bagmode::Write);



for (const auto& pose : poses_hist_copy) {
    bag.write("/pose_history", pose.header.stamp , pose);
    //  std::cout << "pose.header.stamp " << pose.header.stamp << std::endl;
     }
bag.close();

            printf(" Finished transfering all scans saved in buffer \n \n \n ") ;

}



    ~LidarPoseTransformer() {
        // Close the bag file when the node is shutting down
        bag_.close();
    }







private:

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher transformed_cloud_pub_; 
    ros::Publisher odom_publisher;
    ros::Publisher original_cloud_pub_;
    ros::Timer timer_;


    std::queue<std::pair<ros::Time, sensor_msgs::PointCloud2>> cloud_buffer_;
    std::queue<std::pair<ros::Time, geometry_msgs::PoseStamped>> pose_buffer_;

    laser_geometry::LaserProjection projector_;
    filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;

    // tf::TransformListener tf_;
    sensor_msgs::LaserScan scan_filtered_;
    bool ignore_laser_skew_ = true;


 //! \brief Stores history of scans
//   std::deque<sensor_msgs::PointCloud> scan_hist ;
  std::deque< sensor_msgs::LaserScan> scan_hist ;
  boost::mutex scan_hist_mutex_ ;

  //! \brief The number points currently in the scan history
  unsigned int total_pts_ ;

  //! \brief The max number of scans to store in the scan history
  unsigned int max_scans_ = 70000;


//! \brief Specify how much to downsample the data. A value of 1 preserves all the data. 3 would keep 1/3 of the data.
  unsigned int downsample_factor_ = 1;

    std::deque<geometry_msgs::PoseStamped> poses_hist;
    boost::mutex poses_hist_mutex_;
    unsigned int max_poses_ = 70000;
    unsigned int total_poses_;

    bool first_time_stamp_cloud_bool = true;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_assembler_after_loop_closure");
    LidarPoseTransformer transformer;
    ros::spin();
    return 0;
}

