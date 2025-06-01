#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

# imu
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import math
from collections import deque
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np


# import tf2_ros
import tf
import geometry_msgs.msg


class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/sync_scan_cloud_filtered", PointCloud2, queue_size=1)
        # self.laserSub = rospy.Subscriber("/front_laser_link/scan", LaserScan, self.laserCallback) #topicName may be different, please modify it according to the actual situation. You can enter rostopic list in the terminal to view
        self.laserSub = rospy.Subscriber("/front_laser_link/scan", LaserScan, self.laserCallback)  # topicName may be different, please modify it according to the actual situation. You can enter rostopic list in the terminal to view

    def laserCallback(self, data):

        cloud_out = self.laserProj.projectLaser(data)

        self.pcPub.publish(cloud_out)


if __name__ == '__main__':
    rospy.init_node("laser2pointcloud_loam_py")
    l2pc = Laser2PC()

    rospy.spin()
