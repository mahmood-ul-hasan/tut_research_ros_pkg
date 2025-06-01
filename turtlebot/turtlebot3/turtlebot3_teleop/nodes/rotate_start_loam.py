#!/usr/bin/env python3
# license removed for brevity

import rospy
from std_msgs.msg import Float64
import math
import time

rospy.init_node("talker")


import subprocess
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze /catkin_ws/src/latest_crane/laser_to_pcl/src"




def talker():
    position = 0
    pub = rospy.Publisher("/marvin/base_to_laser_joint_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(950) # 10hz
    increment = math.pi/900
    init_time = time.time()

    while not rospy.is_shutdown():
        time_now = time.time() - init_time
        position = position + increment
        text = [position*180/math.pi, time_now]
        pub.publish(position)

        

        # if position > 2*math.pi: 
        if position >= math.pi: 
            # pointcloud to pcd
            # proc = subprocess.Popen(cmd, shell=True)
            # proc.kill()
            # # break
            # position =0
            rospy.loginfo(text)
            print("DOne======================================================")
            increment = -increment
            init_time = time.time()
            

        elif position <= 0.0:
            rospy.loginfo(text)
            increment = -increment
            init_time = time.time()
        
        rate.sleep()



            

if __name__ == '__main__':
    rospy.init_node("talker")

    try:
        talker()
    except rospy.ROSInterruptException:
        pass