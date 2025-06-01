#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def odom_callback(odom, listener):
    try:
        # Convert the odometry message from the base frame to the world frame
        listener.waitForTransform('world', 'camera_odom_frame', rospy.Time(), rospy.Duration(5.0))

        # Create a PoseStamped message for pose2
        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = odom.pose.pose
        # Convert the pose from frame A to frame B

        # Transform the pose from the base frame to the world frame
        transformed_pose = listener.transformPose('world', pose_stamped)

        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.pose.pose = transformed_pose.pose

        # Publish the transformed odometry message
        odom_pub.publish(odom)
        # print(odom)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Failed to transform odometry from camera_odom_frame frame to world frame: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('odom_republisher')

    # Create a TransformListener object
    listener = tf.TransformListener()

    # Create a publisher for the transformed odometry message
    odom_pub = rospy.Publisher('/camera/odom/sample_world_frame', Odometry, queue_size=10)

    # Subscribe to the base frame odometry topic
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback, listener)

    # Spin the ROS node
    rospy.spin()
