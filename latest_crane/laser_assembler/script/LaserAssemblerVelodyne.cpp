#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "LaserAssemblerVelodyne");
    ros::NodeHandle nh;

    // Wait for the service to become available
    ros::service::waitForService("/assemble_scans2");
    ros::ServiceClient assemble_scans_client = nh.serviceClient<laser_assembler::AssembleScans2>("/assemble_scans2");

    // Publisher for the assembled point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/laser_pointcloud_assembler", 1);

    // Set the loop rate
    ros::Rate rate(10);  // 0.2 Hz (5 seconds)

    while (ros::ok()) {
        // Create a service request and response object
        laser_assembler::AssembleScans2 srv;

        // Set the time range for assembling scans
        srv.request.begin = ros::Time(0, 0);
        srv.request.end = ros::Time::now();

        // Call the service
        if (assemble_scans_client.call(srv)) {
            // Log the size of the point cloud data
            ROS_INFO("Got cloud with points: %ld", srv.response.cloud.data.size());

           pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
           pcl::fromROSMsg(srv.response.cloud, *pcl_cloud);
           pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

           pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
           voxel_filter.setInputCloud(pcl_cloud);
           voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f);
           voxel_filter.filter(*filtered_cloud);
           
             // Convert filtered PCL point cloud back to ROS PointCloud2
    sensor_msgs::PointCloud2 ros_filtered_cloud;
    pcl::toROSMsg(*filtered_cloud, ros_filtered_cloud);
 

            // Publish the assembled point cloud
            pub.publish(ros_filtered_cloud);
        } else {
            // Log an error if the service call fails
            ROS_ERROR("Service call failed.");
        }

        // Sleep to maintain the loop rate
        rate.sleep();
    }

    return 0;
}
