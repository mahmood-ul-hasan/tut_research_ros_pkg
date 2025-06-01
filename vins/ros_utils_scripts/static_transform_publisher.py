#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf

if __name__ == '__main__':
    rospy.init_node('static_transform_publisher')

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    transform_stamped = TransformStamped()

    transform_stamped.header.frame_id = 'camera_pose_frame'
    # transform_stamped.header.frame_id = 'body'
    transform_stamped.child_frame_id = 'imu_link'
    transform_stamped.transform.translation.x = 0.0
    transform_stamped.transform.translation.y = 0.0
    transform_stamped.transform.translation.z = 0.0
    euler_angles = (0.0, 0.0, -1.5708)  # Replace with the desired rotation in radians
    quaternion = tf.quaternion_from_euler(*euler_angles)
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    rate = rospy.Rate(300)  # 1 Hz

    while not rospy.is_shutdown():
        transform_stamped.header.stamp = rospy.Time.now()
        tf_broadcaster.sendTransform(transform_stamped)
        print(transform_stamped)
        rate.sleep()
