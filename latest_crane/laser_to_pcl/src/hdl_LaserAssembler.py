#!/usr/bin/env python3

import rospy; 
from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



rospy.init_node("assemble_scans_to_cloud")
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub = rospy.Publisher ("/laser_pointcloud_assembler", PointCloud2, queue_size=1)

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

r = rospy.Rate (1000)

pervious_time = rospy.Time(0,0)
duration = rospy.Duration(2) # duration in sec
pervious_time_rate = rospy.get_rostime() 
pointcloud = PointCloud2()
if __name__ == '__main__':

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():

        curent_time = rospy.get_rostime() 

        try:
            trans = tfBuffer.lookup_transform('world', 'laser', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            r.sleep()
            continue
        odom = Odometry()
        odom.header.stamp =  rospy.Time.now()
        odom.header.frame_id = "world"
        odom.child_frame_id = "laser"
        odom.pose.pose.position = trans.transform.translation
        odom.pose.pose.orientation = trans.transform.rotation
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        # publish the message
        odom_pub.publish(odom)
        print("tf_listener_odm_publisher.py ", rospy.Time.now())
        r.sleep()

        if curent_time.to_nsec() >= pervious_time_rate.to_nsec() + rospy.Duration(.1).to_nsec():
            pervious_time_rate = curent_time
            
            resp = assemble_scans(pervious_time, curent_time)
            
            if curent_time.to_nsec() >= duration.to_nsec():
                pervious_time = curent_time - duration
            print("Got cloud with %u points" % len(resp.cloud.data), "time: ", odom.header.stamp)
            pointcloud = resp.cloud
            pointcloud.header.stamp = odom.header.stamp
            pub.publish (pointcloud)

