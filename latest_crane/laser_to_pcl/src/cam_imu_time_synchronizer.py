#!/usr/bin/env python3
from sensor_msgs.msg import PointCloud2

import message_filters
from std_msgs.msg import Int32, Float32
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import numpy as np
from std_msgs.msg import Float32MultiArray

rospy.init_node('time_synchronizer')
laser_pub = rospy.Publisher("scan_filtered_pointcloud_time_synchro", PointCloud2, queue_size=1)
odom_pub = rospy.Publisher("odom_time_synchro", Odometry, queue_size=50)

i = 0
global imu_time
array = np.array([0.0, 0.0])


def filter_callback(imu0, cam0):
    # The callback processing the pairs of numbers that arrived at approximately the same time
    global i
    i = i + 1

    print("nofil: ", i, "diff_imu_cam: ", imu0.header.stamp - cam0.header.stamp, "imu: ", imu0.header.stamp, "cam: ", cam0.header.stamp)

    # laser_pub.publish (scan_filtered_pointcloud)
    # odom_pub.publish (odom)


def imu_callback(msg):
    global imu_time
    imu_time = msg.header.stamp
    # print(a)


def camera_callback(msg):
    global i
    i = i + 1
    camera_time = msg.header.stamp
    def_time = imu_time - camera_time
    # print(def_time)
    print("no: ", i, msg.header.frame_id, " diff_imu_cam: ", def_time.to_sec())

    array[0] = i
    array[1] = def_time.to_sec()
    array_for_Publish = Float32MultiArray(data=array)
    if msg.header.frame_id == "usb_cam_elp":
        pub_cam_imu_time_diff_elp.publish(array_for_Publish)
    if msg.header.frame_id == "usb_cam_mcm":
        pub_cam_imu_time_diff_mcm.publish(array_for_Publish)


def cal_time_diff(camera_topic, imu_topic):
    rospy.Subscriber(imu_topic, Imu, imu_callback, queue_size=10)
    rospy.Subscriber(camera_topic, Image, camera_callback, queue_size=10)


if __name__ == '__main__':

    pub_cam_imu_time_diff_elp = rospy.Publisher("/pub_cam_imu_time_diff_elp", Float32MultiArray, queue_size=1)
    pub_cam_imu_time_diff_mcm = rospy.Publisher("/pub_cam_imu_time_diff_mcm", Float32MultiArray, queue_size=1)

    camera_topic = "/usb_cam_elp/image_raw"
    imu_topic = "/imu/data"
    cal_time_diff(camera_topic, imu_topic)

    camera_topic = "/usb_cam_mcm/image_raw"
    imu_topic = "/imu/data"
    cal_time_diff(camera_topic, imu_topic)


# filter_msg
    camera_filter_sub = message_filters.Subscriber(camera_topic, Image)
    imu_filter_sub = message_filters.Subscriber(imu_topic, Imu)
    # ts = message_filters.ApproximateTimeSynchronizer([odometry_sub, laser_sub], 10, 0.000000001, allow_headerless=True)
    ts = message_filters.ApproximateTimeSynchronizer([imu_filter_sub, camera_filter_sub], 10, 0.000001, allow_headerless=True)
    ts.registerCallback(filter_callback)
    rospy.spin()
