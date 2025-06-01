#!/usr/bin/env python3
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler





def static_transform_publisher(header, child, x, y, z, r, p, h):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    quat_static =  quaternion_from_euler(r , p, h)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = quat_static[0]
    t.transform.rotation.y = quat_static[1]
    t.transform.rotation.z = quat_static[2]
    t.transform.rotation.w = quat_static[3]
    #print(t)
    br.sendTransform(t)



if __name__ == '__main__':
    rospy.init_node('tf_listener_odm_publisher')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Create a publisher for the PoseStamped message
    odom_pub = rospy.Publisher('odom_crane_structural_info', Odometry, queue_size=10)
    odom_rpy_pub = rospy.Publisher('odom_crane_structural_info_rpy', Odometry, queue_size=10)

  

    flag = True
    
   

    rate = rospy.Rate(500.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'laser', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

            
        # broadcast the tf by rpy of imu
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "crane_info"
        t.transform.translation = trans.transform.translation
        t.transform.rotation = trans.transform.rotation
        br.sendTransform(t)



        tranfered_trans = t.transform.translation
        tranfered_rotate = t.transform.rotation

        # Initialize odometry message
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "crane_info"
        # Fill in the odometry message
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position = tranfered_trans
        odom.pose.pose.orientation = tranfered_rotate
        odom.twist.twist = Twist()

        # Publish the odometry message
        odom_pub.publish(odom)


# Initialize odometry message
        odom_rpy = Odometry()
        odom_rpy.header.frame_id = "world"
        odom_rpy.child_frame_id = "crane_info"
        # Fill in the odometry message
        odom_rpy.header.stamp = rospy.Time.now()
        odom_rpy.pose.pose.position = tranfered_trans

        quaternion = (
            tranfered_rotate.x,
            tranfered_rotate.y,
            tranfered_rotate.z,
            tranfered_rotate.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)

        odom_rpy.pose.pose.orientation.x = rpy[0] * 180 / math.pi
        odom_rpy.pose.pose.orientation.y = rpy[1] * 180 / math.pi
        odom_rpy.pose.pose.orientation.z = rpy[2] * 180 / math.pi
        odom_rpy.twist.twist = Twist()

        # print("rpy: ",  odom_rpy.pose.pose.orientation.x,  odom_rpy.pose.pose.orientation.y,  odom_rpy.pose.pose.orientation.z)

        # Publish the odometry message
        odom_rpy_pub.publish(odom_rpy)

        rate.sleep()



            

