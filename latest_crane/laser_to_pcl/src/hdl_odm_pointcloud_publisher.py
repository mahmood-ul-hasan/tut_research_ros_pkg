#!/usr/bin/env python3  
import rospy


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from sensor_msgs.msg import PointCloud2 
from laser_geometry import LaserProjection

odm_data = Odometry()
odom = Odometry()



j=0




def odomCallback(data):
    global odm_data
    odm_data = data
    # print("recieved = ", odm_data.pose.pose.position)


def laserCallback(data):
    global odm_data
    print("recieved = ", data.header.stamp)
    odom.header.stamp =  data.header.stamp
    odom.header.frame_id = odm_data.header.frame_id
    odom.child_frame_id = odm_data.child_frame_id 
    odom.pose = odm_data.pose
    # pcd_pub.publish(cloud_out)
    odom_pub.publish(odom)



    



if __name__ == '__main__':

    rospy.init_node('odm_pointcloud_publisher')
    print("=============== odm_pointcloud_publisher ====================")

    odom_pub = rospy.Publisher("/odom_crane_structural_info_hf", Odometry, queue_size=50)

    rospy.Subscriber('/odom_crane_structural_info',Odometry,odomCallback)
    rospy.Subscriber("/velodyne_points", PointCloud2, laserCallback)

    rospy.spin()
