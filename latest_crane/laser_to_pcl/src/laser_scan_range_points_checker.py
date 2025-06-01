#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def laser_callback(scan):
    # Check the length of the ranges array
    if len(scan.ranges) == 1441:
        rospy.loginfo("Received LaserScan with 1441 range data points.")
    else:
        rospy.logwarn("Received LaserScan with %d range data points.", len(scan.ranges))

def listener():
    rospy.init_node('laser_scan_checker', anonymous=True)
    rospy.Subscriber("/ld_lrs3611/scan", LaserScan, laser_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

