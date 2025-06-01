#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <filesystem>  


class CropBoxNode
{
private:
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_filter_cloud, pub_filter_cloud_world, pub_odom, pub_path;
    std::string input_topic_;
    std::string filtered_cloud, filtered_cloud_world, odom_topic, path_topic;
    bool filtered_keep_organized_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_, save_rate;
    bool negative_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    int file_index_ = 0;
    std::string output_dir_ = "/media/aisl2/aisl_data/catkin_ws/src/PointCloudProcessing/pointcloud_registration/kobelco_exp_2024/5_pitch_angle_40_and_80_yaw_cycle_slow";
    nav_msgs::Path odom_path; // Declare odom_path as a member variable
    ros::Time last_saved_time ;
    bool isCleanedUp = false;  // Flag to track if cleanup has been done


public:
    CropBoxNode()
    {
        // Node handle
        nh_ = std::make_shared<ros::NodeHandle>("~");

        // Parameters
        nh_->param<std::string>("input", input_topic_, "/velodyne_points");
        nh_->param<std::string>("filtered_cloud", filtered_cloud, "/velodyne_points_filtered");
        nh_->param<std::string>("filtered_cloud_world", filtered_cloud_world, "/velodyne_points_filtered_world");
        nh_->param<std::string>("odom_topic", odom_topic, "/odom_world_to_velodyne");
        nh_->param<std::string>("path_topic", path_topic, "/path_world_to_velodyne");
        nh_->param<bool>("filtered_keep_organized", filtered_keep_organized_, true);
        nh_->param<double>("min_x", min_x_, -22);
        nh_->param<double>("max_x", max_x_, 22);
        nh_->param<double>("min_y", min_y_, -22);
        nh_->param<double>("max_y", max_y_, 22);
        nh_->param<double>("min_z", min_z_, 1.0);
        nh_->param<double>("max_z", max_z_, 100.0);
        nh_->param<double>("save_rate", save_rate, 0.3);
        

        nh_->param<bool>("negative", negative_, true);

        // Subscriber
        sub_ = nh_->subscribe(input_topic_, 1, &CropBoxNode::cloudCallback, this);

        // Publishers
        pub_filter_cloud = nh_->advertise<pcl::PointCloud<pcl::PointXYZI>>(filtered_cloud, 1);
        pub_filter_cloud_world = nh_->advertise<pcl::PointCloud<pcl::PointXYZI>>(filtered_cloud_world, 1);
        pub_odom = nh_->advertise<nav_msgs::Odometry>(odom_topic, 1);
        pub_path = nh_->advertise<nav_msgs::Path>(path_topic, 1);

        // TF Listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    }

    ~CropBoxNode() {}

void cleanupOldFiles()
{
    try
    {
        std::filesystem::path pcd_dir(output_dir_ + "/pcd/");
        for (auto &entry : std::filesystem::directory_iterator(pcd_dir))
        {
            std::filesystem::remove(entry.path());
        }

        std::filesystem::path pcd_world_dir(output_dir_ + "/pcd_world/");
        for (auto &entry : std::filesystem::directory_iterator(pcd_world_dir))
        {
            std::filesystem::remove(entry.path());
        }

        std::string pose_json = output_dir_ + "/pose.json";
        std::filesystem::remove(pose_json);
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Failed to delete old files: %s", e.what());
    }
}



void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
{
     // Perform cleanup only if it hasn't been done yet
        if (!isCleanedUp) {
            cleanupOldFiles();
            isCleanedUp = true;  // Set the flag to true after cleanup
        }



    // Transform the point cloud to a fixed frame
    sensor_msgs::PointCloud2 transformed_cloud;
    geometry_msgs::TransformStamped transform_to_fixed;
    try
    {
        transform_to_fixed = tf_buffer_.lookupTransform("world", input_cloud->header.frame_id, input_cloud->header.stamp, ros::Duration(1.0));
        tf2::doTransform(*input_cloud, transformed_cloud, transform_to_fixed);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    // Extract position (translation)
    double tx = transform_to_fixed.transform.translation.x;
    double ty = transform_to_fixed.transform.translation.y;
    double tz = transform_to_fixed.transform.translation.z;

    // Extract orientation (rotation in quaternion)
    double qw = transform_to_fixed.transform.rotation.w;
    double qx = transform_to_fixed.transform.rotation.x;
    double qy = transform_to_fixed.transform.rotation.y;
    double qz = transform_to_fixed.transform.rotation.z;

    // Create Odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = transform_to_fixed.header.stamp;
    odom_msg.header.frame_id = transform_to_fixed.header.frame_id; // Use header.frame_id instead of header.frame
    odom_msg.child_frame_id = input_cloud->header.frame_id; // Use input_cloud->header.frame_id

    // Set position (translation)
    odom_msg.pose.pose.position.x = tx;
    odom_msg.pose.pose.position.y = ty;
    odom_msg.pose.pose.position.z = tz;

    // Set orientation (rotation)
    odom_msg.pose.pose.orientation.w = qw;
    odom_msg.pose.pose.orientation.x = qx;
    odom_msg.pose.pose.orientation.y = qy;
    odom_msg.pose.pose.orientation.z = qz;

    // Publish the Odometry message
    pub_odom.publish(odom_msg);

    // Add the current pose to the path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    odom_path.poses.push_back(pose_stamped);
    odom_path.header = odom_msg.header;

    // Publish the odometry path
    pub_path.publish(odom_path);


  

    // Convert transformed PointCloud2 to PCL format
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(transformed_cloud, *cloud);

    // CropBox filter
    pcl::CropBox<pcl::PointXYZI> crop_box;
    crop_box.setInputCloud(cloud);
    crop_box.setNegative(negative_);
    crop_box.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0));
    crop_box.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0));

    // Filtered cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    crop_box.filter(*cropped_cloud);

    // Step 4: Convert the cropped cloud back to PointCloud2 format
    sensor_msgs::PointCloud2 cropped_cloud_msg;
    pcl::toROSMsg(*cropped_cloud, cropped_cloud_msg);
    // cropped_cloud_msg.header.frame_id = "world"; // Set the frame to the fixed frame
    cropped_cloud_msg.header.frame_id = "world"; // Set the frame to the fixed frame
    // pub_filter_cloud_world.publish(cropped_cloud_msg);
    pub_filter_cloud_world.publish(transformed_cloud);

    



    // Step 5: Transform the cropped cloud back to the original moving frame
    sensor_msgs::PointCloud2 final_cloud;
    try
    {
        geometry_msgs::TransformStamped transform_to_moving = tf_buffer_.lookupTransform(
            input_cloud->header.frame_id, "world", input_cloud->header.stamp, ros::Duration(1.0));
        tf2::doTransform(cropped_cloud_msg, final_cloud, transform_to_moving);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    // Step 6: Publish the final cropped cloud in the original moving frame
    final_cloud.header.frame_id = input_cloud->header.frame_id;
    final_cloud.header.stamp = input_cloud->header.stamp;
    pub_filter_cloud.publish(final_cloud);

// Convert transformed PointCloud2 to PCL format
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(final_cloud, *final_cloud_pcl);


  // Save pose data to file at a specific rate
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_saved_time).toSec() >= save_rate) 
    {
        std::string odom_filename = output_dir_ + "/pose.json";
        std::ofstream file(odom_filename, std::ios::app);
        if (file.is_open()) {
            file << tx << " " << ty << " " << tz << " "
                 << qw << " " << qx << " " << qy << " " << qz << "\n";
            file.close();
        } else {
            ROS_ERROR("Failed to open file for saving pose data: %s", odom_filename.c_str());
        }
    



        // Save the cropped cloud as PCD
        std::string filename = output_dir_ + "/pcd/" + std::to_string(file_index_) + ".pcd";
        if (pcl::io::savePCDFileASCII(filename, *final_cloud_pcl) == 0)
        {
            ROS_INFO("PointCloud saved to %s", filename.c_str());
        }
        else
        {
            ROS_ERROR("Failed to save PointCloud to %s", filename.c_str());
        }

        // Save the cropped cloud as PCD
        std::string filename_world = output_dir_ + "/pcd_world/" + std::to_string(file_index_) + ".pcd";
        if (pcl::io::savePCDFileASCII(filename_world, *cropped_cloud) == 0)
        {
            ROS_INFO("PointCloud saved to %s", filename_world.c_str());
        }
        else
        {
            ROS_ERROR("Failed to save PointCloud to %s", filename_world.c_str());
        }

        // Increment file index
        file_index_++;



        last_saved_time = current_time; // Update the last saved time

}
}

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "crop_box_filter");
    CropBoxNode crop_box_node;
    ros::spin();
    return 0;
}
