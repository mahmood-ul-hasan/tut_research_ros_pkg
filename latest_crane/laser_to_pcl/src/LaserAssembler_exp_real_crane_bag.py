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
import sys
from sensor_msgs.msg import JointState
import os
import signal
t2 = time.time()
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


from threading import Thread



# laser assembler
print('waiting for lasser_assembler service')
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub_point_cloud = rospy.Publisher ("/laser_pointcloud_optimze", PointCloud2, queue_size=1)

# point clould to pcd command
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/"

imu_data =  np.zeros((36*100, 7))
orient_at_t = np.array([0, 0, 0, 0, 0, 0])
i=0



global proc
global position
position = 0
clockwise =0
anticlockwise = 0
clockwise_flag = 1
global ii
ii = 0


def Position_state(msg):
    
    global position
    global ii
    global flag
    global pervious_time
    global proc

    quat_upper = (msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
    euler_upper = euler_from_quaternion(quat_upper)
    yaw = math.floor(euler_upper[2]*180/math.pi)
    roll = math.floor(euler_upper[0]*180/math.pi)
    pitch = math.floor(euler_upper[1]*180/math.pi)
    
    position =  180-yaw 
    # print("a", ii, math.floor(roll), math.floor(pitch), "msg_pos ", math.floor(yaw), "pos ", math.floor(position))
    ii = ii +1



# def iiiiii():
    
    current_time = time.time()

    # print("b", current_time - pervious_time, "msg_pos ", math.floor(yaw), "pos ", math.floor(position))


    if current_time - pervious_time >= 3:
        pervious_time = current_time 

        # if position >=0 and position < 360 and flag == 1: 

        # laser assembler
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        print ("Got cloud with points" , len(resp.cloud.data)) 
        #print(resp.cloud.data)
        pub_point_cloud.publish (resp.cloud)
        # print("pos (position >=0 and position <= 350)", position, flag)



        # rotate the base joint
        # print("Pub =====")
        # print("Rotating base position: ", position)

    
    if position >= 350 and position < 355 and flag == 1: 
        # pointcloud to pcd
        # proc = subprocess.Popen(cmd, shell=True)
        # proc = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)
        print("Stating recieving PCD ======================================================")
        # flag =0
        # print("pos (position >= 350 and position <= 355)", position, flag)


    if position >= 355:
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        pub_point_cloud.publish (resp.cloud)

        # proc.kill()              # when ready to shutdown:
        # proc.terminate()              # when ready to shutdown:
        # os.kill(proc.pid, signal.SIGINT)
        # flag =0
        print("pos (position >= 355)", position, flag)
        print("Stoping recieving ==========================================================")
        print("  ")
        print("  ")
        print("  ")
        print("=====================================================================")
        print("Scan cycle is complete; map is saved; Relaunch to again start scaning")
        print("=====================================================================")
        print("  ")
        print("  ")
        print("  ")
        print("exit")
        # sub.unregister()



        

    

        

if __name__ == '__main__':
    
    print("Starting...... LaserAssembler_exp_real_crane ")

    rospy.init_node("LaserAssembler_exp_real_crane")
    print("Starting...... LaserAssembler_exp_real_crane ")


    rate = rospy.Rate(500) 
    global flag 
    flag = 1
    time_begin = rospy.Time.now()
    t1 = time.time()
    # rate.sleep()
    global pervious_time

    pervious_time = time.time()



    sub = rospy.Subscriber("/imu/quaternion", QuaternionStamped, Position_state)
    # sub = rospy.Subscriber("/imu_boom_link/quaternion", QuaternionStamped, Position_state)

    rospy.spin();    # Added this line






    # try:
    # except rospy.ROSInterruptException:
    #     pass