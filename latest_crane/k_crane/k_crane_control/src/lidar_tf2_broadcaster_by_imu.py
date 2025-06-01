#!/usr/bin/env python  
# license removed for brevity

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg


# imu
from std_msgs.msg import String
from sensor_msgs.msg import Imu
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

##########################################################################
# tf for laser by gazebo link
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

# quat to rpy
i=0
j=0
array = np.array([0.0, 0.0, 0.0, 0.0])
array1 = np.array([0.0, 0.0, 0.0, 0.0])

from gazebo_msgs.srv import GetModelState
import rospy

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Tutorial:

    _blockListDict = {
        'block_a': Block('dtw_robot', 'boom_link'),
        #'block_b': Block('brick_box_3x1x3', 'chassis'),

    }

    def show_gazebo_models(self):
        global laser_position
        global laser_pose
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for block in self._blockListDict.itervalues():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                print '\n'
                print 'Status.success = ', resp_coordinates.success
                print(blockName)
                print("Cube " + str(block._name))
                print("Valeur de X : " + str(resp_coordinates.pose.position))
                print("Quaternion X : " + str(resp_coordinates.pose.orientation))
                # laser_position = [resp_coordinates.pose.position.x, resp_coordinates.pose.position.y, resp_coordinates.pose.position.z]
                laser_pose = [resp_coordinates.pose.orientation.x, resp_coordinates.pose.orientation.y, resp_coordinates.pose.orientation.z, resp_coordinates.pose.orientation.w]


        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))



#################################################################################




   # Convert quat to rpy
    # quaternion = (
    # msg.orientation.x,
    # msg.orientation.y,
    # msg.orientation.z,
    # msg.orientation.w)
    # euler = euler_from_quaternion(quaternion)

    # quat = quaternion_from_euler(0, 0,euler[2])
    # print(quat)


def imu_upper_link_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "lower_link"
    t.child_frame_id = "upper_link"

    # Convert quat to rpy
    quat_upper = (
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w)
    euler_upper = euler_from_quaternion(quat_upper)

    quat_upper_yaw = quaternion_from_euler(0, 0,euler_upper[2])
    #print(quat_upper_yaw)



    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.3
    t.transform.rotation.x = quat_upper_yaw[0]
    t.transform.rotation.y = quat_upper_yaw[1]
    t.transform.rotation.z = quat_upper_yaw[2]
    t.transform.rotation.w = quat_upper_yaw[3]
  
    #print(t)
    br.sendTransform(t)


    #=====================================================
    # quat to rpy conversion and publish rpy
    global i
    i=i+1
    # rospy.loginfo("I heared %s", msg.orientation)
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (quat_upper_yaw)

    array[0]= roll*180/math.pi
    array[1]= pitch*180/math.pi
    array[2]=yaw*180/math.pi
    array[3]=i
    #print("array \n",array)


    array_forPublish = Float32MultiArray(data=array)
    pub_rpy_upper_link.publish(array_forPublish)



  
     

def imu_boom_link_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()

    t.header.frame_id = "upper_link"
    t.child_frame_id = "boom_link"

     # Convert quat to rpy
    quat_boom = (
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w)
    euler_boom = euler_from_quaternion(quat_boom)

    quat_boom_yaw = quaternion_from_euler(0, euler_boom[1], 0,)
    print(quat_boom_yaw)



    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.3
    t.transform.rotation.x = quat_boom_yaw[0]
    t.transform.rotation.y = quat_boom_yaw[1]
    t.transform.rotation.z = quat_boom_yaw[2]
    t.transform.rotation.w = quat_boom_yaw[3]
    #print(t)
    br.sendTransform(t)


     #=====================================================
    # quat to rpy conversion and publish rpy
    global j
    j=j+1
    # rospy.loginfo("I heared %s", msg.orientation)
    orientation_q1 = msg.orientation
    orientation_list1 = [orientation_q1.x, orientation_q1.y, orientation_q1.z, orientation_q1.w]
    (roll1, pitch1, yaw1) = euler_from_quaternion (quat_boom_yaw)
    # print("roll ", roll*180/math.pi, "pitch", pitch*180/math.pi, "yaw", yaw*180/math.pi)

    array1[0]= roll1*180/math.pi
    array1[1]= pitch1*180/math.pi
    array1[2]=yaw1*180/math.pi
    array1[3]=j

    array_forPublish1 = Float32MultiArray(data=array1)
    pub_rpy_boom_link.publish(array_forPublish1)

     


def imu_hook_link_callback(msg):

    #tuto = Tutorial()
    #tuto.show_gazebo_models()

      # static trasform
    static_transform_publisher("world", "base_link")
    static_transform_publisher("base_link", "lower_link")
    # static_transform_publisher("body_link", "front_laser_link")




    
    #print(msg)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()


    q = quaternion_from_euler(msg.position[0], 0, 0)
  

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "boom_link"
    t.child_frame_id = "front_laser_link"
    t.transform.translation.x =  3
    t.transform.translation.y = 0
    t.transform.translation.z =  0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    # print(msg.position[0])
    br.sendTransform(t)

 ############################################################
    # t.transform.rotation.x = laser_pose[0]
    # t.transform.rotation.y = laser_pose[1]
    # t.transform.rotation.z = laser_pose[2]
    # t.transform.rotation.w = laser_pose[3]

    # t.transform.rotation.x = 0
    # t.transform.rotation.y = 0
    # t.transform.rotation.z = 0
    # t.transform.rotation.w = 1
  ############################################################

   



def static_transform_publisher(header, child):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    # print(t)
    br.sendTransform(t)




if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_by_imu')   
    pub_rpy_upper_link = rospy.Publisher("/rpy_imu_upper_link", Float32MultiArray, queue_size=1)
    pub_rpy_boom_link = rospy.Publisher("/rpy_imu_boom_link", Float32MultiArray, queue_size=1)

    rospy.Subscriber("/imu_upper_link/data", Imu, imu_upper_link_callback)

    # rospy.Subscriber("/marvin/joint_states", JointState, imu_hook_link_callback, queue_size=10)
    rospy.Subscriber("/marvin/joint_states", JointState,  imu_hook_link_callback)
    rospy.Subscriber("/imu_boom_link/data", Imu, imu_boom_link_callback)

    rospy.spin()


    #try:
       # main_()
    #except rospy.ROSInterruptException:
        #pass