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
import time

if __name__ == '__main__':
    rospy.init_node("talker")

  
    time_begin =  rospy.get_time()
    t1 = time.time()
    time.sleep(1)
    time_end =  rospy.get_time()
    t2 = time.time()

    print("ttt", t2)
    print("t: {0}".format(time.time() - t1))
    print("t: {0}".format(t2))


    duration = time_end - time_begin
    print("dur: {0}".format(duration))

    rospy.loginfo("Slept for " + str(time_end) + " secs")
    # rospy.loginfo("duration time %i %i", duration.secs, duration.nsecs)

    print((time_begin) )
    print((time_end) )
    print((duration) )
    