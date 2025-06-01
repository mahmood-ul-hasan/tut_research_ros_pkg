#!/usr/bin/env python3
# license removed for brevity

import subprocess
import rospy
from std_msgs.msg import Float64
import math
import time

rospy.init_node("talker")


cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze /catkin_ws/src/latest_crane/laser_to_pcl/src"


def talker():
    # position = -math.pi / 2
    position = 180 * math.pi / 180  # Initial position set to -80 degrees

    pub = rospy.Publisher("/marvin/base_to_laser_joint_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(9)  # 10hz
    increment = math.pi / 900

    # rate = rospy.Rate(90) # 10hz
    # increment = math.pi/(900*8)

    init_time = time.time()
    counter = 0

    while not rospy.is_shutdown():
        time_now = time.time() - init_time
        position = position + increment
        text = [round(position * 180 / math.pi), time_now]
        pub.publish(position)
        # rospy.loginfo(text)

        # if position > 2*math.pi:
        if position > 180 * math.pi / 180:
            counter += 1

            # pointcloud to pcd
            # proc = subprocess.Popen(cmd, shell=True)
            # proc.kill()
            # # break
            # position =0
            rospy.loginfo(text)
            print("DOne======================================================", counter)
            increment = -increment
            init_time = time.time()

        elif position < 0 * math.pi / 180:
            counter += 1
            rospy.loginfo(text)
            increment = -increment
            init_time = time.time()
            print("DOne-------------------------------------------------------", counter)

            # break

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("talker")

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
