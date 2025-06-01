#!/usr/bin/env python3

# Import required ROS libraries
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import message_filters

# Initialize the ROS node
rospy.init_node('time_synchronizer')

# Publishers for synchronized topics
laser_pub = rospy.Publisher("velodyne_points_time_synchro", PointCloud2, queue_size=10)
odom_pub = rospy.Publisher("odom_crane_structural_info_time_synchro", Odometry, queue_size=10)

# Callback function for synchronized messages
def filter_callback(odom, scan_filtered_pointcloud):
    # Log the timestamps for debugging
    # rospy.loginfo(f"Received synchronized messages: "
    #               f"Odom time: {odom.header.stamp}, "
    #               f"PointCloud2 time: {scan_filtered_pointcloud.header.stamp}")

    # Publish synchronized messages
    laser_pub.publish(scan_filtered_pointcloud)
    odom_pub.publish(odom)

if __name__ == '__main__':
    # Subscribers for the input topics
    laser_sub = message_filters.Subscriber("/velodyne_points", PointCloud2)
    odometry_sub = message_filters.Subscriber("/odom_crane_structural_info_hf", Odometry)
    
    # Approximate Time Synchronizer
    ts = message_filters.ApproximateTimeSynchronizer(
        [odometry_sub, laser_sub],  # List of topics to synchronize
        queue_size=100,              # Buffer size for messages
        slop=0.000000001                    # Allowed time difference (in seconds)
    )

    ts.registerCallback(filter_callback)

    # Keep the ROS node running
    rospy.spin()
