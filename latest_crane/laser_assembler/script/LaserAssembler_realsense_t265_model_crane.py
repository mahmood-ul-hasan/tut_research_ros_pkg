#!/usr/bin/env python
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

t2 = time.time()


# laser assembler
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub_point_cloud = rospy.Publisher("/laser_pointcloud_optimze", PointCloud2, queue_size=1)

# point clould to pcd command
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/"


pervious_time = time.time()
position = 0


clockwise = 0
anticlockwise = 0
clockwise_flag = 1


def Position_state(msg):
    global clockwise_flag
    global clockwise
    global anticlockwise
    global pervious_time

    # print("msg_pos", msg.position[0], clockwise_flag, clockwise, anticlockwise,)

    if clockwise_flag == 1:
        if msg.position[0] < 100:
            clockwise = 1
        elif msg.position[0] > 250:
            anticlockwise = 1
    clockwise_flag = 0

    if clockwise == 1:
        position = msg.position[0] - 90
    if anticlockwise == 1:
        position = 270 - msg.position[0]
    # print("pos", math.floor(position), clockwise, anticlockwise, msg.position[0])

    if position >= 0 and position < 179:
        current_time = time.time()

        if current_time > pervious_time + (10):
            pervious_time = current_time
            # laser assembler
            resp = assemble_scans(rospy.Time(0, 0), rospy.get_rostime())
            print("Got cloud with %u points" % len(resp.cloud.data))
            # print(resp.cloud.data)
            pub_point_cloud.publish(resp.cloud)

            # rotate the base joint
            # print("Pub =====")


if __name__ == '__main__':

    print("Starting...... LaserAssembler_exp_crane ")

    rospy.init_node("LaserAssembler_real_crane")
    print("Starting...... LaserAssembler_real_crane ")
    rospy.Subscriber("/orion_rotating_base/joint_states", JointState, Position_state, queue_size=10)

    rospy.spin()    # Added this line

    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass
