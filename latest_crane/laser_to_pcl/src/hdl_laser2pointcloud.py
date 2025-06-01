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
j= 0




# conversion laser to PC
class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/scan_filtered_pointcloud", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan_filtered", LaserScan, self.laserCallback) #topicName may be different, please modify it according to the actual situation. You can enter rostopic list in the terminal to view

    def laserCallback(self,data):

        cloud_out = self.laserProj.projectLaser(data)

        self.pcPub.publish(cloud_out)
        # print("recieved = ", cloud_out.data)
        global j
        j=j+1
        # print("=============== Laser to PC ====================  = ", j+1)
        print("Got cloud with %u points" % len(cloud_out.data), "time: ", cloud_out.header.stamp)




   
# main  

if __name__ == '__main__':
    rospy.init_node("laser2pointcloud")
    l2pc = Laser2PC()
    print("=============== Main Main ====================")
    rospy.spin()

