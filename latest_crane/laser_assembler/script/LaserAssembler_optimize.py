#!/usr/bin/env python3
# license removed for brevity
import time

import rospy
from std_msgs.msg import Float64
import math

from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2
import subprocess
import numpy as np
from numpy import savetxt
from sensor_msgs.msg import Imu


t2 = time.time()



rospy.init_node("talker")

# laser assembler
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub_point_cloud = rospy.Publisher ("/laser_pointcloud_optimze", PointCloud2, queue_size=1)
r = rospy.Rate (5)

# point clould to pcd command
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/"
imu_data =  np.zeros((36*100, 7))
orient_at_t = np.array([0, 0, 0, 0, 0, 0])
i=0
pervious_time = time.time()




kkk = 0
# fun to save imu data with specefic sample data frequency
def imu_ListenerCallback(msg):
    global i, np_imu_data, pervious_time

    #current_time =  rospy.get_rostime()
    current_time = time.time()
    if current_time >  pervious_time +  (0.01):
        i=i+1

        if (i >= 36*100):
            imu_data.resize(i+1, 7)
        # rospy.loginfo("I heared %s", msg.orientation)
        quat_orient = msg.orientation
        orient_at_t = np.array([msg.header.stamp.to_sec(),  msg.header.stamp.secs, msg.header.stamp.nsecs, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]) 
        imu_data[i] = orient_at_t[:]
        
        pervious_time = current_time 


def talker():
    position = 0
    pub_rotate = rospy.Publisher("/marvin/base_to_laser_joint_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(10) 
    flag = 1
    time_begin = rospy.Time.now()
    t1 = time.time()


    ii =0

    while not rospy.is_shutdown():
    # while ii == 1:

        position = position + math.pi/180
        if position < (2*math.pi + math.pi/5):
            rospy.loginfo(position*180/math.pi)


        if position >=0 and position < 2*math.pi + math.pi/15: 

            # laser assembler
            resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            #print "Got cloud with %u points" % len(resp.cloud.data)
            #print(resp.cloud.data)
            pub_point_cloud.publish (resp.cloud)

            # rotate the base joint
            pub_rotate.publish(position)
            rate.sleep()
            print("Pub =====")

            sub = rospy.Subscriber("/imu_upper_link/data", Imu, imu_ListenerCallback)

        
        if position >= 2*math.pi and flag == 1: 
            # pointcloud to pcd
            proc = subprocess.Popen(cmd, shell=True)
            print("DOne======================================================")
            flag =0
            sub.unregister()
            np.savetxt('/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/imu_data.csv', imu_data, delimiter=',', fmt = '%.9f')
            print("t1: {0}".format(time.time() - t1))
            print("t2: {0}".format(time.time() - t2))
            print(rospy.Time.now() - time_begin)
            



        if position >= (2*math.pi + math.pi/26) and position <= (math.pi + math.pi/5):
            print("killing ====================")
            proc.kill()              # when ready to shutdown:
            sub.unregister()



        
            

        

if __name__ == '__main__':
    rospy.init_node("talker")


    try:
        talker()
    except rospy.ROSInterruptException:
        pass