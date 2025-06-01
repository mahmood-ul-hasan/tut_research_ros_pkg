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


rospy.init_node("talker")

# laser assembler
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub_point_cloud = rospy.Publisher("/laser_pointcloud_assembler", PointCloud2, queue_size=1)
r = rospy.Rate(5)

# point clould to pcd command
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/"


def talker():
    position = 0
    pub_rotate = rospy.Publisher("/marvin/base_to_laser_joint_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(10)
    flag = 1

    while not rospy.is_shutdown():
        # while ii == 1:

        position = position + math.pi / 180

        if position < (2 * math.pi + math.pi / 5):
            rospy.loginfo(position * 180 / math.pi)

        if position >= 0 and position < 2 * math.pi + math.pi / 15:

            # laser assembler
            resp = assemble_scans(rospy.Time(0, 0), rospy.get_rostime())
            # print "Got cloud with %u points" % len(resp.cloud.data)
            # print(resp.cloud.data)
            pub_point_cloud.publish(resp.cloud)

            # rotate the base joint
            pub_rotate.publish(position)
            rate.sleep()
            print("Pub =====")

        if position >= 2 * math.pi and flag == 1:
            # pointcloud to pcd
            proc = subprocess.Popen(cmd, shell=True)
            print("DOne======================================================")
            flag = 0

        # if position >= (2*math.pi + math.pi/26) and position <= (math.pi + math.pi/5):
        if position >= (2 * math.pi + math.pi / 26):
            print("killing ====================")
            proc.kill()              # when ready to shutdown:
            position = 0
            # break


if __name__ == '__main__':
    rospy.init_node("talker")

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
