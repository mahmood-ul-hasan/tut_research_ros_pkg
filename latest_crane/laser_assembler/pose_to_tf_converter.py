#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

class PoseSubscriber:
    def __init__(self):
        rospy.init_node('pose_subscriber_node', anonymous=True)

        # Create a tf2_ros buffer and broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to the pose topic
        self.pose_sub = rospy.Subscriber('/poses', PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        # Create a transform from "world" to "laser" based on the received pose
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "laser"
        transform.transform.translation = msg.pose.position
        transform.transform.rotation = msg.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        print(transform)

if __name__ == '__main__':
    try:
        pose_subscriber = PoseSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
