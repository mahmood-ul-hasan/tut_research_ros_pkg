#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Float64
import math

rospy.init_node("talker")


import subprocess
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze /catkin_ws/src/latest_crane/laser_to_pcl/src"




def talker():
    position = 0
    pub = rospy.Publisher("/marvin/base_to_laser_joint_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        position = position + math.pi/180
        rospy.loginfo(position*180/math.pi)
        pub.publish(position)
        rate.sleep()

        

        if position >= 2*math.pi: 
            # pointcloud to pcd
            #proc = subprocess.Popen(cmd, shell=True)
            #proc.kill()
            break
            print("DOne======================================================")
            position = 0
            # when ready to shutdown:
            

if __name__ == '__main__':
    rospy.init_node("talker")

    try:
        talker()
    except rospy.ROSInterruptException:
        pass