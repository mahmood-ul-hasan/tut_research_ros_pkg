#!/usr/bin/env python3


import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2, LaserScan
import math 


def process_scan_data():
    try:
        resp = assemble_scans(rospy.Time(0, 0), rospy.get_rostime())
        print("Got cloud with %u points" % len(resp.cloud.data))
        pub.publish(resp.cloud)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def laser_scan_callback(scan_data):
    scan_ranges_size = len(scan_data.ranges)
    print(scan_ranges_size, end=" , ")
    has_nan = any(math.isnan(value) for value in scan_data.ranges)
    has_inf = any(math.isinf(value) for value in scan_data.ranges)
    
    if has_nan:
        print("Found NaN values in scan_data.ranges")
    if has_inf:
        print("Found infinite values in scan_data.ranges")
    # print("scan_ranges_size: ", scan_ranges_size)


rospy.init_node("LaserAssembler_crane_structural_info")
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub = rospy.Publisher("/laser_pointcloud_assembler", PointCloud2, queue_size=1)
rospy.Subscriber("/front_laser_link/scan", LaserScan, laser_scan_callback)  # Replace with your laser scan topic

r = rospy.Rate(.05)

stored_scan_data = None

while not rospy.is_shutdown():
    process_scan_data()
    # r.sleep()
