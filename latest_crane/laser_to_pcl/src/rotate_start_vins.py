#!/usr/bin/env python3
# license removed for brevity

import rospy
from std_msgs.msg import Float64
import math

rospy.init_node("talker")


import subprocess
# cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/"
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/sync_scan_cloud_filtered _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/"




def talker():
    position = 0
    pub = rospy.Publisher("/marvin/base_to_laser_joint_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(200) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        position = position + math.pi/900
        rospy.loginfo(position*180/math.pi)
        pub.publish(position)
        rate.sleep()

        

        if position >= 2*math.pi: 
            # pointcloud to pcd
            proc = subprocess.Popen(cmd, shell=True)
            proc.kill()
            print("DOne======================================================")
            position = 0
            # break
            # when ready to shutdown:
            

if __name__ == '__main__':
    rospy.init_node("talker")

    try:
        talker()
    except rospy.ROSInterruptException:
        pass