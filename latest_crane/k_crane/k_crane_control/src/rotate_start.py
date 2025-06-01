#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Float64
import math

rospy.init_node("talker")

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
            print("DOne======================================================")
            position = 0

if __name__ == '__main__':
    rospy.init_node("talker")
    try:
        talker()
    except rospy.ROSInterruptException:
        pass