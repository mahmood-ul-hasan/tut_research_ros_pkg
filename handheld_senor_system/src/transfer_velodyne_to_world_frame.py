#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
import tf2_geometry_msgs
import tf_conversions
import math

def pointcloud_callback(pointcloud_msg):
    global tf_buffer, tf_listener

    try:
        # Look up the transform from the 'rotation_base' frame to the 'velodyne' frame
        transform = tf_buffer.lookup_transform('rotation_base', 
                                               pointcloud_msg.header.frame_id, 
                                               rospy.Time(0),  # Get the latest available transform
                                               rospy.Duration(0.01))  # Adjust as needed

        # Transform the PointCloud2 message from the 'velodyne' frame to the 'rotation_base' frame
        transformed_cloud = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)

        # Publish the transformed PointCloud2 message
        pointcloud_pub.publish(transformed_cloud)
    
    except tf2_ros.LookupException as e:
        rospy.logwarn("Transform lookup error: {}".format(e))
    except tf2_ros.ExtrapolationException as e:
        rospy.logwarn("Transform extrapolation error: {}".format(e))

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pointcloud_transformer')

    # Create a buffer and listener for tf2 transforms
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a publisher for the transformed PointCloud2 message
    pointcloud_pub = rospy.Publisher('/velodyne_points_world', PointCloud2, queue_size=10)

    # Subscribe to the /velodyne_points topic
    rospy.Subscriber('/velodyne_points', PointCloud2, pointcloud_callback)

    # Keep the node running
    rospy.spin()
