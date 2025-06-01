#! /usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
import math
import matplotlib.pyplot as plt
import time

# def my_publisher():
# control part


if __name__ == '__main__':

    rospy.init_node('move_crane')
    control_publisher = rospy.Publisher('/k_crane/k_crane_joint_controller/command', JointTrajectory, queue_size=10)

    yaw_pos = -1

    pitch_pos = 0
    pitch_increment = 0.1
    rate = rospy.Rate(1)

    # fig = plt.figure()
    t = 0
    int_time = time.time()

    pitch_cycle = True
    yaw_start_point = 0
    pitch_start_point = 0
    init_time = time.time()

    while not rospy.is_shutdown():

        time_now = time.time() - init_time

        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.joint_names = ['pitch_joint', 'yaw_joint']

        point = JointTrajectoryPoint()

        point.positions = [yaw_pos * math.pi / 180, pitch_pos * math.pi / 180]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = rospy.Duration(1)

        msg.points.append(point)
        control_publisher.publish(msg)
        # rospy.loginfo( msg )

        print("pitch = ", round(pitch_pos,2), "yaw = ", round(yaw_pos, 2), )

        pitch_pos = pitch_pos - pitch_increment
        text = ["Rotation angle = ", round(pitch_pos, 2), " Time = ", round(time_now, 2)]

        # if (pitch_pos > 175):
        if (pitch_pos > 10):
            pitch_increment = -pitch_increment
            rospy.loginfo(text)
            init_time = time.time()
            # rospy.sleep(20.0)  # Sleep for 1 second
            # rospy.sleep(15.0)  # Sleep for 1 second

        # elif(pitch_pos <= -175):
        elif(pitch_pos <= -10):
            pitch_increment = - pitch_increment
            rospy.loginfo(text)
            init_time = time.time()
            # rospy.sleep(20.0)  # Sleep for 1 second
            # rospy.sleep(15.0)  # Sleep for 1 second


        rate.sleep()
