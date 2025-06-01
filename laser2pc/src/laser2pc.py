#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

# imu 
from std_msgs.msg import String
from sensor_msgs.msg import Imu
i=0
j=0

# imu subscribe and print
def imu_ListenerCallback(msg):
    rospy.loginfo("I heared %s", msg.orientation)
    global i
    i=i+1
    print("=============== IMU ==================== = ", i+1)
    

def imu_Listener(): 
    rospy.Subscriber("/imu_hook_down/data", Imu, imu_ListenerCallback)


# conversion laser to PC
class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloudSingle", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/front_laser_link/scan", LaserScan, self.laserCallback) #topicName may be different, please modify it according to the actual situation. You can enter rostopic list in the terminal to view

    def laserCallback(self,data):

        cloud_out = self.laserProj.projectLaser(data)

        self.pcPub.publish(cloud_out)
        global j
        j=j+1
        print("=============== Laser to PC ====================  = ", j+1)



   
# main  

if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    l2pc = Laser2PC()
    imu_Listener()
    print("=============== Main Main ====================")
    rospy.spin()

