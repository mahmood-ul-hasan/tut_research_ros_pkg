#!/usr/bin/env python3
# license removed for brevity

import rospy

# Because of transformations
#import tf_conversions

import tf2_ros
import geometry_msgs.msg



# imu
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import math
from collections import deque
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np
import rosparam


from tf.transformations import *


# tf for laser by gazebo link
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from gazebo_msgs.srv import GetModelState
pervious_time = 0.0
pervious_time1= 0.0





def imu_hook_link_callback(msg):
    global pervious_time1


    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
  

    q = quaternion_from_euler(msg.position[0], 0*math.pi/180, 0) 

    t.header.stamp = rospy.Time.now()
    current_time = rospy.Time.now().to_sec()
    t.header.frame_id = "rotating_base"
    t.child_frame_id = "front_laser_link"
    t.transform.translation.x =  0
    t.transform.translation.y = 0
    t.transform.translation.z =  0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    print(msg.position[0]*180/math.pi)
    
    if current_time > pervious_time1 + 0.01 :
     br.sendTransform(t)

    pervious_time1 = current_time
   
   
   
   
    static_transform_publisher("camera_odom_frame", "rotating_base" , 0.0 , 0.0, 0.0,   0, 0, 0)


   



def static_transform_publisher(header, child, x, y, z, r, p, h):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    quat_static =  quaternion_from_euler(r , p, h)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = quat_static[0]
    t.transform.rotation.y = quat_static[1]
    t.transform.rotation.z = quat_static[2]
    t.transform.rotation.w = quat_static[3]
    #print(t)
    br.sendTransform(t)





if __name__ == '__main__':


    rospy.init_node('tf2_broadcaster_loam')   
    rospy.Subscriber("/marvin/joint_states", JointState, imu_hook_link_callback, queue_size=10)

    rospy.spin()



   



