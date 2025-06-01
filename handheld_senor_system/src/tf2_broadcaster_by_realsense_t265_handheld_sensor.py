#!/usr/bin/env python  
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
import math

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

import rosparam



from tf.transformations import *


# tf for laser by gazebo link
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from gazebo_msgs.srv import GetModelState

global ii

# quat to rpy
i=0
ii=0
j=0


global rotating_base
global t_hokoyo, t_velodyne
global flag
flag = False

pervious_time = 0.0


def imu_callback(msg):
        global t_hokoyo, t_velodyne
        global flag
        global pervious_time

        if flag == True:
            br_velodune = tf2_ros.TransformBroadcaster()
            br_hokydo = tf2_ros.TransformBroadcaster()
            t_hokoyo.header.stamp = rospy.Time.now()
            t_velodyne.header.stamp = rospy.Time.now()
            
            current_time = rospy.Time.now().to_sec()
            if current_time > pervious_time + 0.0000:

                br_velodune.sendTransform(t_velodyne)
                br_hokydo.sendTransform(t_hokoyo)
                # static_transform_publisher("rotation_base", "laser",  0.04, 0.19, .06,    180*math.pi/180, 90*math.pi/180, 0*math.pi/180)
                static_transform_publisher("camera_pose_frame", "rotation_base",  0, 0, 0,    0*math.pi/180, 0*math.pi/180, 90*math.pi/180)
                static_transform_publisher("camera_init", "camera_odom_frame",  0.0, 0.0, 0.0,    0*math.pi/180, 0*math.pi/180, 0*math.pi/180)
                # static_transform_publisher("world", "camera_init",  0.0, 0.0, 0.0,    0*math.pi/180, 0*math.pi/180, 0*math.pi/180)


            pervious_time = current_time

            # print("-------------------", t.header.stamp, " ", flag)
            # print(flag)







def rotating_base_link_callback(msg):

      # laser is placed by rotation of 90 along y axis
    global t_hokoyo, t_velodyne , flag, rotating_base, ii


    # q_hokoyo = quaternion_from_euler(90*math.pi/180, 0*math.pi/180, (msg.position[0])*math.pi/180) 
    q_hokoyo =  quaternion_from_euler(90*math.pi/180, -90*math.pi/180, (msg.position[0])*math.pi/180) 
    br_hokydo = tf2_ros.TransformBroadcaster()
    t_hokoyo = geometry_msgs.msg.TransformStamped()
    t_hokoyo.header.stamp = rospy.Time.now()
    t_hokoyo.header.frame_id = "rotation_base"
    t_hokoyo.child_frame_id = "laser"
    t_hokoyo.transform.translation.x = 0.00
    t_hokoyo.transform.translation.y = 0.06
    t_hokoyo.transform.translation.z = 0.06
    t_hokoyo.transform.rotation.x = q_hokoyo[0]
    t_hokoyo.transform.rotation.y = q_hokoyo[1]
    t_hokoyo.transform.rotation.z = q_hokoyo[2]
    t_hokoyo.transform.rotation.w = q_hokoyo[3]




    q_velodyne = quaternion_from_euler(-90*math.pi/180, 0*math.pi/180, (msg.position[0])*math.pi/180) 

    br_velodune = tf2_ros.TransformBroadcaster()
    t_velodyne = geometry_msgs.msg.TransformStamped()
    t_velodyne.header.stamp = rospy.Time.now()
    t_velodyne.header.frame_id = "rotation_base"
    t_velodyne.child_frame_id = "velodyne"
    t_velodyne.transform.translation.x = 0.00
    t_velodyne.transform.translation.y = -0.04
    t_velodyne.transform.translation.z = 0.06
    t_velodyne.transform.rotation.x = q_velodyne[0]
    t_velodyne.transform.rotation.y = q_velodyne[1]
    t_velodyne.transform.rotation.z = q_velodyne[2]
    t_velodyne.transform.rotation.w = q_velodyne[3]
    print(t_velodyne)


    flag = False

    br_velodune.sendTransform(t_velodyne)
    br_hokydo.sendTransform(t_hokoyo)


    flag = True

    print("Pos-90= ",ii , round(msg.position[0]-90, 1), " ", flag)
    #print("Pos = ",msg.position[0])
 

    rotating_base = msg.position[0]-90
   
    ii = ii +1



  
   



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
    rospy.init_node('tf2_broadcaster_by_real_sense_imu')   
    print("hello Mahmod")

    
    pub_rpy_upper_link = rospy.Publisher("/rpy_imu_boom_link", Float32MultiArray, queue_size=5)
    # pub_rpy_boom_link = rospy.Publisher("/rpy_imu_boom_link", Float32MultiArray, queue_size=1)
    
    
    rospy.Subscriber("/camera/imu", Imu, imu_callback)
 

    
    # Below is used rotaing_base
    rospy.Subscriber("/orion_rotating_base/joint_states", JointState, rotating_base_link_callback, queue_size=5)


  

    rospy.spin()
    print("hello Mahmod")



    #try:
       # main_()
    #except rospy.ROSInterruptException:
        #pass







