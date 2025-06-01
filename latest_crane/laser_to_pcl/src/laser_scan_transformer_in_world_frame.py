#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import math
class LaserScanTransformer:
    def __init__(self):
        rospy.init_node('laser_scan_transformer')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.scan_sub = rospy.Subscriber('/ld_lrs3611/scan_filtered', LaserScan, self.scan_callback)
        self.scan_pub = rospy.Publisher('/ld_lrs3611/scan_filtered_world', LaserScan, queue_size=10)

    def scan_callback(self, scan_msg):
        try:
            # Lookup the transformation from the laser frame to the world frame
            transform = self.tf_buffer.lookup_transform('world', scan_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            
            # Prepare the transformed LaserScan message
            scan_world_msg = LaserScan()
            scan_world_msg.header.stamp = scan_msg.header.stamp
            scan_world_msg.header.frame_id = 'world'
            scan_world_msg.angle_min = scan_msg.angle_min
            scan_world_msg.angle_max = scan_msg.angle_max
            scan_world_msg.angle_increment = scan_msg.angle_increment
            scan_world_msg.time_increment = scan_msg.time_increment
            scan_world_msg.scan_time = scan_msg.scan_time
            scan_world_msg.range_min = scan_msg.range_min
            scan_world_msg.range_max = scan_msg.range_max

            scan_world_msg.ranges = []
            scan_world_msg.intensities = []

            for i, range in enumerate(scan_msg.ranges):
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = range * math.cos(angle)
                y = range * math.sin(angle)

                point_laser = PointStamped()
                point_laser.header.frame_id = scan_msg.header.frame_id
                point_laser.header.stamp = scan_msg.header.stamp
                point_laser.point.x = x
                point_laser.point.y = y
                point_laser.point.z = 0.0

                point_world = tf2_geometry_msgs.do_transform_point(point_laser, transform)

                range_world = math.sqrt(point_world.point.x**2 + point_world.point.y**2)
                scan_world_msg.ranges.append(range_world)
                if scan_msg.intensities:
                    scan_world_msg.intensities.append(scan_msg.intensities[i])

            self.scan_pub.publish(scan_world_msg)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform error: %s", e)

if __name__ == '__main__':
    try:
        transformer = LaserScanTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
