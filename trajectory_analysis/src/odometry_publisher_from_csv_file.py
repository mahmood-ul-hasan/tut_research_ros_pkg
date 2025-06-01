#!/usr/bin/env python3

import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_inverse
from tf.transformations import quaternion_multiply
import time
import rospy
import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import QuaternionStamped
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import sys

if __name__ == '__main__':
    rospy.init_node('odometry_publisher_from_csv_file')
    odom_pub = rospy.Publisher('/odom_motion_cap_sys', Odometry, queue_size=10)

    # Define the CSV file path
    csv_file_path = "/home/aisl2/catkin_ws/src/data/model_crane_data/23_10_13_model_crane_data_to_comapre_different_methods_with_ground_truth/random_3_quaternion.csv"
    
    # Check if a command-line argument is provided
    # if len(sys.argv) != 2:
    #     print("Usage: <csv_file_path>")
    #     sys.exit(1)

    # Get the CSV file path from the command-line argument
    # csv_file_path = sys.argv[1]


    publishing_rate = rospy.Rate(100)  # Publish at 10 Hz

    # rospy.sleep(1.5)
    # rospy.sleep(4)  # traj 1
    # rospy.sleep(1.6)  # traj 2
    rospy.sleep(3.2)  # traj 3
    # rospy.sleep(4.6)  # traj 4
    # rospy.sleep(2)  # traj 5

    # Open the CSV file and read the data
    with open(csv_file_path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)  # Skip the header row if present
        next(csvreader)  # Skip the header row if present
        row_init = next(csvreader)  # Read the next row
        indx, time_stmp, init_quat_x, init_quat_y, init_quat_z, init_quat_w, init_pos_x, init_pos_y, init_pos_z = map(float, row_init)

        initial_quaternion = [init_quat_x, init_quat_y, init_quat_z, init_quat_w]
        roll, pitch, yaw = euler_from_quaternion(initial_quaternion)
        print("init ", roll * 180 / math.pi, pitch * 180 / math.pi, yaw * 180 / math.pi)

        initial_inverse = quaternion_inverse(initial_quaternion)
        inv_roll, inv_pitch, inv_yaw = euler_from_quaternion(initial_inverse)
        print("init ", inv_roll * 180 / math.pi, inv_pitch * 180 / math.pi, inv_yaw * 180 / math.pi)

        while not rospy.is_shutdown():

            row = next(csvreader)  # Read the next row
            # Parse the CSV data
            indx, time_stmp, quat_x, quat_y, quat_z, quat_w, pos_x, pos_y, pos_z = map(float, row)

#------------------------
# subtract initial values
            pos_x = pos_x - init_pos_x
            pos_y = pos_y - init_pos_y
            pos_z = pos_z - init_pos_z

            # Define the subsequent quaternion
            # subsequent_quaternion = Quaternion(quat_x, quat_y, quat_z, quat_w)
            subsequent_quaternion = [quat_x, quat_y, quat_z, quat_w]

            inv_roll, inv_pitch, inv_yaw = euler_from_quaternion(subsequent_quaternion)
            print("subsequent_quaternion ", round(inv_roll * 180 / math.pi, 3), round(inv_pitch * 180 / math.pi, 3), round(inv_yaw * 180 / math.pi, 3))
            print(rospy.Time.now())

            # Calculate the relative rotation
            relative_rotation = quaternion_multiply(initial_inverse, subsequent_quaternion)
            inv_roll, inv_pitch, inv_yaw = euler_from_quaternion(relative_rotation)
            # print("relative_rotation     ", round(inv_roll * 180 / math.pi, 3), round(inv_pitch * 180 / math.pi, 3), round(inv_yaw * 180 / math.pi, 3))
#-------------------------------------

            # Create an Odometry message
            odom_msg = Odometry()
            odom_msg.header = Header()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom_mot_cap_sys"

            # Fill in the Pose information (excluding velocity fields)
            odom_msg.pose.pose.position.x = pos_x
            odom_msg.pose.pose.position.y = pos_y
            odom_msg.pose.pose.position.z = pos_z
            # odom_msg.pose.pose.orientation = Quaternion(x=relative_rotation[0], y=relative_rotation[1], z=relative_rotation[2], w=relative_rotation[3])
            odom_msg.pose.pose.orientation = Quaternion(x=subsequent_quaternion[0], y=subsequent_quaternion[1], z=subsequent_quaternion[2], w=subsequent_quaternion[3])

            # Publish the Odometry message
            odom_pub.publish(odom_msg)

            publishing_rate.sleep()
