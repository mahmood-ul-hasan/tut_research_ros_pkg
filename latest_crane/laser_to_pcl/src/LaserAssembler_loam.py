#!/usr/bin/env python3
# license removed for brevity
import time

import rospy
import rospy
from std_msgs.msg import Float64
import math

from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2
import subprocess
import numpy as np
from numpy import savetxt
from sensor_msgs.msg import Imu
import roslaunch





rospy.init_node("LaserAssembler_mloam")

# laser assembler
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub_point_cloud = rospy.Publisher ("/laser_cloud_last_2", PointCloud2, queue_size=1)
r = rospy.Rate (5)

# point clould to pcd command
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/"







def talker():
    position = -math.pi/2
    pub_rotate = rospy.Publisher("/marvin/base_to_laser_joint_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(90) 
    increment = math.pi/900
    # init_time = time.time()
    init_time=rospy.get_time()

    flag = 1

  



    while not rospy.is_shutdown():
    # while ii == 1:

        time_now = rospy.get_time() - init_time
        position = position + increment
        text = [position*180/math.pi, time_now]
        pub_rotate.publish(position)
        rospy.loginfo(text)
        # print("time.time() = ", rospy.get_time(), "init_time = ", init_time)




        if position >= math.pi/2: 
            # pointcloud to pcd
            # proc = subprocess.Popen(cmd, shell=True)
            # proc.kill()
            # # break
            # position =0
            rospy.loginfo(text)
            print("DOne======================================================")
            increment = -increment
            resp = assemble_scans(rospy.Time.from_sec(init_time), rospy.get_rostime())
            init_time = rospy.get_time()
            pub_point_cloud.publish (resp.cloud)


            
            

        elif position <= -math.pi/2:
            rospy.loginfo(text)
            increment = -increment
            resp = assemble_scans(rospy.Time.from_sec(init_time), rospy.get_rostime())
            init_time = rospy.get_time()
            pub_point_cloud.publish (resp.cloud)
            # break 
        
        rate.sleep()


        
            



        
            

        

if __name__ == '__main__':


    try:
        talker()
    except rospy.ROSInterruptException:
        pass