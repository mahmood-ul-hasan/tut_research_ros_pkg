#!/usr/bin/env python3  
# license removed for brevity

import rospy

# Because of transformations
#import tf_conversions

import tf2_ros
import geometry_msgs.msg

# base is 1.1 meter
# IMU is  0.9 meter

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

laser_position = np.array([0.0, 0.0, 0.0, 0.0])
laser_pose = np.array([0.0, 0.0, 0.0, 0.0])
q1_inv = np.array([0.0, 0.0, 0.0, 0.0])
from tf.transformations import *


# tf for laser by gazebo link
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from gazebo_msgs.srv import GetModelState

# quat to rpy
i=0

j=0
array = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
array1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
boom_link_default_joint = 10*math.pi/180
upper_link_default_joint = 0*math.pi/180

global rotating_base
rotating_base = 0

global ii
ii=0



#################################################################################
def imu_upper_link_callback(msg):

    # Convert quat to rpy
    quat_upper = (
    msg.quaternion.x,
    msg.quaternion.y,
    msg.quaternion.z,
    msg.quaternion.w)
    euler_upper = euler_from_quaternion(quat_upper)
    # publish rpy
    global i
    i=i+1
    array[0]= euler_upper[0]*180/math.pi
    array[1]= euler_upper[1]*180/math.pi
    array[2]= euler_upper[2]*180/math.pi
    array[3]=rotating_base
    array[4]=i
    #print("array \n",array)
    array_forPublish = Float32MultiArray(data=array)
    pub_rpy_upper_link.publish(array_forPublish)
    #-------------------------------------------------------
# 
    # broadcast the tf by rpy of imu
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "lower_link"
    t.child_frame_id = "upper_link"


    quat_upper_yaw = quaternion_from_euler(0, 0, euler_upper[2])
    # quat_upper_yaw = quaternion_from_euler(0, 0, 0)
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = quat_upper_yaw[0]
    t.transform.rotation.y = quat_upper_yaw[1]
    t.transform.rotation.z = quat_upper_yaw[2]
    t.transform.rotation.w = quat_upper_yaw[3]
    #print(t)
    br.sendTransform(t)

    
    
    print("-u",  math.floor(euler_upper[1]*180/math.pi) ,  math.floor(euler_upper[2]*180/math.pi),  math.floor(rotating_base),ii)
    #print("Pos = ",msg.position[0])
    br.sendTransform(t)


    #ii = ii +1

####################################################################









##==============================================================================

def imu_boom_link_callback(msg):

    # Convert quat to rpy
    quat_boom = (
    msg.quaternion.x,
    msg.quaternion.y,
    msg.quaternion.z,
    msg.quaternion.w)
    euler_boom = euler_from_quaternion(quat_boom)
    # publish the rpy
    global j
    j=j+1
    array1[0]= euler_boom[0]*180/math.pi
    array1[1]= euler_boom[1]*180/math.pi
    array1[2]=euler_boom[2]*180/math.pi
    array1[3]=rotating_base
    array1[4]=j
    #array_forPublish1 = Float32MultiArray(data=array1)
    #pub_rpy_boom_link.publish(array_forPublish1)
    #------------------------------------------------------------

   
    # broadcast the tf by rpy of imu
    # quat_boom_yaw = quaternion_from_euler(euler_boom[0], euler_boom[1]  , euler_boom[2] - boom_link_default_joint)
    print("-b",  math.floor(euler_boom[1]*180/math.pi) ,  math.floor(euler_boom[2]*180/math.pi),  math.floor(rotating_base),j)

    #print(quat_boom_yaw)
    
    quat_boom_yaw =  quaternion_from_euler(0, ( euler_boom[1]), 0)
    # quat_boom_yaw =  quaternion_from_euler(0, 0, 0)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "upper_link"
    t.child_frame_id = "boom_link"
    t.transform.translation.x =  0
    t.transform.translation.y = 0
    t.transform.translation.z = 0.33
    t.transform.rotation.x = quat_boom_yaw[0]
    t.transform.rotation.y = quat_boom_yaw[1]
    t.transform.rotation.z = quat_boom_yaw[2]
    t.transform.rotation.w = quat_boom_yaw[3]

    
    # print("  ",j,  euler_boom[1]*180/math.pi ,  euler_boom[2]*180/math.pi,  rotating_base)
    #print("Pos = ",msg.position[0])
    br.sendTransform(t)

    global ii
    #ii = ii +1



def rotating_base_link_callback(msg):

      # laser is placed by rotation of 90 along y axis
    #q = quaternion_from_euler(-0*math.pi/180 , 90*math.pi/180, -(msg.position[0] -90)*math.pi/180) 
    q = quaternion_from_euler(0, 0, -(msg.position[0] -90)*math.pi/180) 

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "boom_link"
    t.child_frame_id = "rotation_base_link"
    t.transform.translation.x = 1.234
    t.transform.translation.y = -0.08
    t.transform.translation.z = 0.0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
   # print("Pos-90= ",ii , msg.position[0]-90)
    #print("Pos = ",msg.position[0])
    br.sendTransform(t)

    global rotating_base
    rotating_base = msg.position[0]-90
    global ii
    ii = ii +1

    
    # print("  ",ii,  rotating_base)

    # Dimension of crane frame
    #lower link height = 0.3
    #base link height = 0.8 m
    # boom link height = 0.3 m
    # rotating base  link height = 1.234 m
      #  t.transform.translation.x = -1.234
      #  t.transform.translation.y = -0.08

    static_transform_publisher("world", "base_link" , 0.0 , 0.0, 0.8,   0, 0, 0)
    static_transform_publisher("base_link", "lower_link" , 0.0 , 0.0, 0.28,   0, 0, 0)
    # static_transform_publisher("lower_link", "boom_link" , 0.0 , 0.0, 0.28,   0, 0, 0)
    
    # static_transform_publisher("rotation_base_link", "laser",  0, 0, 0,    0*math.pi/180, 90*math.pi/180, 0*math.pi/180)
    static_transform_publisher("rotation_base_link", "laser",  0, 0, 0,    0*math.pi/180, 90*math.pi/180, 90*math.pi/180)

    # Dimension of Map
    # total y axis 384
    # y +ve => 130
    # y -ve => 260
    # total x axis 380
    # x +ve => 184
    # x -ve => 195



    #=====================================================
    

   



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
    rospy.init_node('tf2_broadcaster_by_imu')   
    print("hello Mahmod")

    
    pub_rpy_upper_link = rospy.Publisher("/rpy_imu_boom_link1", Float32MultiArray, queue_size=1)

    rospy.Subscriber("/orion_rotating_base/joint_states", JointState, rotating_base_link_callback, queue_size=10)
    rospy.Subscriber("/imu/quaternion", QuaternionStamped, imu_boom_link_callback)
    rospy.Subscriber("/imu/quaternion", QuaternionStamped, imu_upper_link_callback)
    # Below is used rotaing_base

  

    rospy.spin()
    print("hello Mahmod")



    #try:
       # main_()
    #except rospy.ROSInterruptException:
        #pass







