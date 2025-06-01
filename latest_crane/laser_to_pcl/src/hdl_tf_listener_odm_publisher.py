#!/usr/bin/env python3  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)


if __name__ == '__main__':
    rospy.init_node('tf_listener_odm_publisher')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

   

    rate = rospy.Rate(10000000.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'laser', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        odom = Odometry()
        odom.header.stamp =  rospy.Time.now()
        odom.header.frame_id = "world"

        # set the position

        # odom.pose.pose.position = Pose(Point(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z), Quaternion(trans.transform.rotation.x))
        odom.pose.pose.position = trans.transform.translation
        odom.pose.pose.orientation = trans.transform.rotation
        # print(odom.pose.pose)

        # set the velocity
        odom.child_frame_id = "laser"
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        # publish the message
        odom_pub.publish(odom)
        print("tf_listener_odm_publisher.py ", rospy.Time.now())



        rate.sleep()
