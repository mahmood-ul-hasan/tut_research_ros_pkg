#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Float64
import math

from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2
import subprocess
import numpy as np
from numpy import savetxt
from sensor_msgs.msg import Imu



rospy.init_node("talker")

# laser assembler
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub_point_cloud = rospy.Publisher ("/laser_pointcloud_optimze", PointCloud2, queue_size=1)
r = rospy.Rate (5)

# point clould to pcd command
cmd = "rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/"
imu_data =  np.array([0, 0, 0, 0, 0, 0])
orient_at_t = np.array([0, 0, 0, 0, 0, 0])
i=0



def imu_ListenerCallback(msg):
    global i
    i=i+1
    # rospy.loginfo("I heared %s", msg.orientation)
    quat_orient = msg.orientation
    orient_at_t = ([msg.header.stamp.secs, msg.header.stamp.nsecs, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]) 
    np.insert(imu_data, 0, orient_at_t)
    print("imu = ", imu_data)



def talker():
    position = 0
    pub_rotate = rospy.Publisher("/marvin/base_to_laser_joint_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(5) # 10hz
    r = rospy.Rate(1) # 10hz
    flag = 1

    while not rospy.is_shutdown():

        position = position + math.pi/180
        if position < (math.pi + math.pi/5):
            rospy.loginfo(position*180/math.pi)



        if position >=0 and position < math.pi + math.pi/15: 

            # laser assembler
            resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            print "Got cloud with %u points" % len(resp.cloud.data)
            #print(resp.cloud.data)
            pub_point_cloud.publish (resp.cloud)

            # rotate the base joint
            pub_rotate.publish(position)
            rate.sleep()
            print("Pub =====")

            # msg = rospy.wait_for_message("my_topic", MyType)
            msg = rospy.wait_for_message("/imu_upper_link/data", Imu)
            imu_ListenerCallback(msg)


            # sub = rospy.Subscriber("/imu_upper_link/data", Imu, imu_ListenerCallback)








        

        if position >= math.pi and flag == 1: 
            # pointcloud to pcd
            proc = subprocess.Popen(cmd, shell=True)
            print("DOne======================================================")
            flag =0
            # sub.unregister()
            savetxt('imu_data.csv', imu_data, delimiter=',')



        if position >= (math.pi + math.pi/26) and position <= (math.pi + math.pi/5):
            print("killing ====================")
            proc.kill()              # when ready to shutdown:


        
            

            
            
            
            

if __name__ == '__main__':
    rospy.init_node("talker")



    try:
        talker()
    except rospy.ROSInterruptException:
        pass