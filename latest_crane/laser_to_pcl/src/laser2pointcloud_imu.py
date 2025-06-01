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


i=0
j=0
roll = 1
pitch = 2
yaw = 3
orient = []


#================================================================================
# imu subscribe and print
rospy.init_node("laser2pointcloud")

pub_orient = rospy.Publisher("/orientation", Float32MultiArray, queue_size=1)
#rpy = Float32MultiArray()
#rpy.size = 3
#rpy.label = 'rpy'

array = np.array([0.0, 0.0, 0.0, 0.0])

   

def imu_Listener(): 
    rospy.Subscriber("/imu_hook_down/data", Imu, imu_ListenerCallback)


def imu_ListenerCallback(msg):
    global i
    i=i+1
    # rospy.loginfo("I heared %s", msg.orientation)
    orientation_q = msg.orientation
    global roll
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # print("roll ", roll*180/math.pi, "pitch", pitch*180/math.pi, "yaw", yaw*180/math.pi)

    array[0]= roll*180/math.pi
    array[1]= pitch*180/math.pi
    array[2]=yaw*180/math.pi
    array[3]=i
    #array.append(roll)
    #array.append(pitch)
    #array.append(yaw)
    print("array \n",array)

    #rpy.data =array
    array_forPublish = Float32MultiArray(data=array)
    #array_forPublish = Float32MultiArray(rpy)
    pub_orient.publish(array_forPublish)

   
 

    # print("roll ", roll*180/math.pi, "pitch", pitch*180/math.pi, "yaw", yaw*180/math.pi)
   
    #print("=============== IMU ==================== = ", i+1)
    



#================================================================================
# conversion laser to PC
class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laser_pointcloud_single2", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/front_laser_link/scan", LaserScan, self.laserCallback) #topicName may be different, please modify it according to the actual situation. You can enter rostopic list in the terminal to view

    def laserCallback(self,data):

        cloud_out = self.laserProj.projectLaser(data)

        self.pcPub.publish(cloud_out)
        global j
        j=j+1
        #print("=============== Laser to PC ====================  = ", j+1)



   
# main  

if __name__ == '__main__':
    rospy.init_node("laser2pointcloud")
    l2pc = Laser2PC()
    imu_Listener()
    print("=============== Main Main ====================")
    #plt.plot(roll)
    #plt.ylabel('some numbers')
    #plt.show()
    print("roll",roll)
  

    rospy.spin()

