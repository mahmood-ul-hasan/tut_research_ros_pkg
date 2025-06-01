#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
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
        transformed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/transformed_cloud", 10);
        original_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/original_cloud", 1);
        odom_publisher = nh_.advertise<nav_msgs::Odometry>("/pose", 1);
        odom_publisher = nh_.advertise<nav_msgs::Odometry>("/odom", 1);


        timer_ = nh_.createTimer(ros::Duration(60.0), &LidarPoseTransformer::periodicTask, this);
    }

    
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        
        // Convert to PCL data type
        laser_geometry::LaserProjection projector_;
        sensor_msgs::PointCloud cur_cloud;
        // sensor_msgs::PointCloud2 cur_cloud2;
        projector_.projectLaser(*scan_msg, cur_cloud);



        //  sensor_msgs::convertPointCloudToPointCloud2(cur_cloud, cur_cloud2 );
        // std::cout << "first: " << cur_cloud << std::endl;



        original_cloud_pub_.publish(cur_cloud);

        // Add the current scan (now of type PointCloud) into our history of scans
        scan_hist_mutex_.lock() ;
        if (scan_hist.size() == max_scans_)                           // Is our deque full?
        {
        total_pts_ -= scan_hist.front().points.size () ;            // We're removing an elem, so this reduces our total point count
        scan_hist.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it
        }
        scan_hist.push_back(cur_cloud) ;                              // Add the newest scan to the back of the deque
        total_pts_ += cur_cloud.points.size () ;                       // Add the new scan to the running total of points

        // printf("Scans: %4u  Points: %10u\n", scan_hist.size(), total_pts_) ;

        scan_hist_mutex_.unlock() ;
        ROS_DEBUG("done with msgCallback");
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
        std::deque<sensor_msgs::PointCloud> scan_hist_copy = scan_hist;
        std::deque<geometry_msgs::PoseStamped> poses_hist_copy = poses_hist;

   auto pose_it = poses_hist_copy.begin();
        const ros::Duration time_diff = scan_hist.front().header.stamp - poses_hist_copy.front().header.stamp;


// store in bag file
rosbag::Bag bag;
bag.open("/media/aisl2/aisl_data/catkin_ws/src/latest_crane/laser_assembler/data.bag", rosbag::bagmode::Write);

for (const auto& scan : scan_hist_copy) {
    bag.write("/scan_history", scan.header.stamp, scan);}
for (const auto& pose : poses_hist_copy) {
    bag.write("/pose_history", pose.header.stamp , pose);}
bag.close();



     
        // Iterate over poses
        while (pose_it != poses_hist_copy.end()) {

            // Publish the current pose
            geometry_msgs::PoseStamped& current_pose = *pose_it;
            
            // Create a new odometry message
            nav_msgs::Odometry odom_msg;
            odom_msg.header = current_pose.header;
            odom_msg.pose.pose = current_pose.pose;
            odom_publisher.publish(odom_msg);


            static tf2_ros::TransformBroadcaster broadcaster;
            // Create a transform message
            geometry_msgs::TransformStamped transformStamped;
            // Fill in the header
            // transformStamped.header.stamp = current_pose.header.stamp;
            transformStamped.header.stamp =     ros::Time::now();
            transformStamped.header.frame_id = "world";  // Parent frame ID
            transformStamped.child_frame_id = "laser";   // Child frame ID
            transformStamped.transform.translation.x = current_pose.pose.position.x;
            transformStamped.transform.translation.y = current_pose.pose.position.y;
            transformStamped.transform.translation.z = current_pose.pose.position.z;
            transformStamped.transform.rotation = current_pose.pose.orientation;
            broadcaster.sendTransform(transformStamped);



            // Get the timestamp of the current pose
            const ros::Time& current_pose_time = current_pose.header.stamp;


        std::cout << " scan_time " << scan_hist_copy.front().header.stamp << ", pose_time " << current_pose_time + time_diff ;
        // Iterate over the copied lidar scans and publish those whose timestamps are less than the timestamp of the current pose
        
        int scan_size_before = scan_hist_copy.size();
        printf(",  Scans_buffur_size_BEFORE_transfer: %4u  ", scan_hist_copy.size()) ;
        while (!scan_hist_copy.empty() && scan_hist_copy.front().header.stamp < current_pose_time + time_diff)
         {
            // Publish lidar cloud
            sensor_msgs::PointCloud lidar_cloud = scan_hist_copy.front();
            // transformed_cloud_pub_.publish(lidar_cloud);

            tranform_cloud_by_pose(current_pose, lidar_cloud );
            // printf(" %4u , ", scan_hist_copy.size()) ;

            // Remove the used scan from the copy
            scan_hist_copy.pop_front();

                broadcaster.sendTransform(transformStamped);
        }

        int scan_size_after = scan_hist_copy.size();

        printf("Scans_buffur_size_AFTER_transfer: %4u , ", scan_hist_copy.size()) ;
        printf(" No_of_scans_transfered_using_current_pose : %4u , \n ", scan_size_before - scan_size_after) ;

        // Move to the next pose
        ++pose_it;
    }

            printf(" Finished transfering all scans saved in buffer \n \n \n ") ;

}

void tranform_cloud_by_pose(geometry_msgs::PoseStamped& current_pose, sensor_msgs::PointCloud closest_cloud){

// Extract translation and quaternion from PoseStamped message
Eigen::Vector3f translation(current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            current_pose.pose.position.z);
Eigen::Quaternionf quaternion(current_pose.pose.orientation.w,
                               current_pose.pose.orientation.x,
                               current_pose.pose.orientation.y,
                               current_pose.pose.orientation.z);

// Construct transformation matrix
Eigen::Affine3d transform_matrix = Eigen::Affine3d::Identity();
transform_matrix.translation() = translation.cast<double>();
transform_matrix.linear() = quaternion.toRotationMatrix().cast<double>();


// Assuming closest_cloud is of type sensor_msgs::PointCloud
sensor_msgs::PointCloud2 closest_cloud2;
sensor_msgs::PointCloud2 transformed_cloud2;

// Convert sensor_msgs::PointCloud to sensor_msgs::PointCloud2
sensor_msgs::convertPointCloudToPointCloud2(closest_cloud, closest_cloud2);

// Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg(closest_cloud2, *cloud_ptr);

// Transform point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, transform_matrix);

// Convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
pcl::toROSMsg(*transformed_cloud_ptr, transformed_cloud2);


// Modify the frame ID of the transformed point cloud
transformed_cloud2.header.frame_id = "world";

sensor_msgs::PointCloud transformed_cloud;
sensor_msgs::convertPointCloud2ToPointCloud(transformed_cloud2, transformed_cloud);


// Publish the transformed point cloud
transformed_cloud_pub_.publish(transformed_cloud);


}
void matchPosesAndClouds() {
    // Lock both mutexes using lock guards
    boost::lock_guard<boost::mutex> scan_lock(scan_hist_mutex_);
    boost::lock_guard<boost::mutex> poses_lock(poses_hist_mutex_);


// Extract x, y, and z coordinates from each PoseStamped
std::vector<double> x_coordinates, y_coordinates, z_coordinates;
for (const auto& pose : poses_hist) {
    x_coordinates.push_back(pose.pose.position.x);
    y_coordinates.push_back(pose.pose.position.y);
    z_coordinates.push_back(pose.pose.position.z);
}

// Plot x vs y
matplotlibcpp::plot(x_coordinates, y_coordinates);
matplotlibcpp::xlabel("X");
matplotlibcpp::ylabel("Y");
// matplotlibcpp::show();


    // Container to store transformed clouds
    std::deque<sensor_msgs::PointCloud2> transformed_clouds;

    // Iterate through each pose in poses_hist_
    for (const auto& pose : poses_hist) {
        // Find the closest cloud in scan_hist based on timestamp
        auto closest_cloud_iter = std::min_element(scan_hist.begin(), scan_hist.end(),
            [&pose](const sensor_msgs::PointCloud& cloud1, const sensor_msgs::PointCloud& cloud2) {
                // Calculate absolute time difference between pose and cloud timestamps
                ros::Duration diff1 = pose.header.stamp - cloud1.header.stamp;
                ros::Duration diff2 = pose.header.stamp - cloud2.header.stamp;
                return std::abs(diff1.toSec()) < std::abs(diff2.toSec());
            });

        // Do something with the matched pose and cloud
        if (closest_cloud_iter != scan_hist.end()) {
            // Match found, do something with the pose and closest cloud
            sensor_msgs::PointCloud closest_cloud = *closest_cloud_iter;


// Extract translation and quaternion from PoseStamped message
Eigen::Vector3f translation(pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z);
Eigen::Quaternionf quaternion(pose.pose.orientation.w,
                               pose.pose.orientation.x,
                               pose.pose.orientation.y,
                               pose.pose.orientation.z);

// Construct transformation matrix
Eigen::Affine3d transform_matrix = Eigen::Affine3d::Identity();
transform_matrix.translation() = translation.cast<double>();
transform_matrix.linear() = quaternion.toRotationMatrix().cast<double>();


// Assuming closest_cloud is of type sensor_msgs::PointCloud
sensor_msgs::PointCloud2 closest_cloud2;
sensor_msgs::PointCloud2 transformed_cloud2;

// Convert sensor_msgs::PointCloud to sensor_msgs::PointCloud2
sensor_msgs::convertPointCloudToPointCloud2(closest_cloud, closest_cloud2);

// Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg(closest_cloud2, *cloud_ptr);

// Transform point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, transform_matrix);

// Convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
pcl::toROSMsg(*transformed_cloud_ptr, transformed_cloud2);

transformed_cloud2 = closest_cloud2;

// Modify the frame ID of the transformed point cloud
transformed_cloud2.header.frame_id = "world";

sensor_msgs::PointCloud transformed_cloud;
sensor_msgs::convertPointCloud2ToPointCloud(transformed_cloud2, transformed_cloud);


// Publish the transformed point cloud
transformed_cloud_pub_.publish(transformed_cloud);




// // Transform cloud
// sensor_msgs::PointCloud2 transformed_cloud2;
// // tf2::doTransform(closest_cloud2, transformed_cloud2, transform_stamped);



        }
    }
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
  std::deque<sensor_msgs::PointCloud> scan_hist ;
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

