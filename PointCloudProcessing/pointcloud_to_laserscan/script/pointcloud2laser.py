#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import math
def pointcloud2_to_laserscan(point_cloud):
    # Initialize the LaserScan message
    laser_scan = LaserScan()
    laser_scan.header = point_cloud.header
    laser_scan.angle_min = -4.71238899230957  # -90 degrees
    laser_scan.angle_max =   4.71238899230957# 90 degrees
    laser_scan.angle_increment = 0.004363323096185923  # Angle increment
    laser_scan.range_min = 2  # Minimum range value
    laser_scan.range_max = 250  # Maximum range value
    num_ranges = int((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment)
    laser_scan.ranges = [float('inf')] * num_ranges

    # Convert the PointCloud2 message to a list of points
    points = list(pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True))

    # Populate the LaserScan ranges
    for point in points:
        x, y, z = point[:3]
        print("x, y, z ", x, " ",  y, " ", z)
        if x == 0 and y == 0:  # Skip invalid points
            print("---------------------------x, y, z ", x, " ",  y, " ", z)
            continue
        angle = math.atan2(y, x)

        if laser_scan.angle_min <= angle <= laser_scan.angle_max:
            range_idx = int((angle - laser_scan.angle_min) / laser_scan.angle_increment)
            distance = math.sqrt(x ** 2 + y ** 2)
            print("angle ", angle)


            if laser_scan.range_min <= distance <= laser_scan.range_max:
                laser_scan.ranges[range_idx] = min(laser_scan.ranges[range_idx], distance)
                print("distance ", distance)


    return laser_scan

def pointcloud_callback(point_cloud):
    # Convert PointCloud2 to LaserScan
    laser_scan = pointcloud2_to_laserscan(point_cloud)
    # Publish the LaserScan message
    laserscan_pub.publish(laser_scan)

if __name__ == '__main__':
    rospy.init_node('pointcloud2_to_laserscan')
    pointcloud_sub = rospy.Subscriber('/ld_lrs3611/scan_filtered_pc', PointCloud2, pointcloud_callback)
    laserscan_pub = rospy.Publisher('/ld_lrs3611/scan_filtered_world22', LaserScan, queue_size=10)
    rospy.spin()

