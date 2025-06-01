#!/usr/bin/env python3

import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import math
import tf.transformations as tf
import pcl_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import atexit
from rospy.service import ServiceException

resp = None

prev_yaw = None
direction = None
last_change_time = None
# start_time = rospy.Time.now()  # Start time for assembling scans

# Initialize global variables
yaw_angles = []
pitch_angles = []
roll_angles = []
timestamps = []

prev_yaw = None  # To store the previous yaw value
cycle_count = 0  # To count the number of completed cycles

crossed_zero_pos = False
crossed_zero_neg = False
start_time = None
ZERO_THRESHOLD = 6  # Define a small range around zero


# Time threshold to ignore sudden changes (in seconds)
# TIME_THRESHOLD = 50 # time for 4
TIME_THRESHOLD = 30 # time for 5

def save_final_map():
    global resp  # Declare resp as global
    global start_time, map_count
    if resp.cloud is not None and len(resp.cloud.data) > 0:
        filename = f'capture000{map_count:02}.pcd'
        save_point_cloud_pcd(resp.cloud, filename)
        print(f"Final map saved as {filename}")
        map_count += 1
 

def normalize_angle(angle):
    """ Normalize angle to be within -pi to pi """
    return math.atan2(math.sin(angle), math.cos(angle))


def save_point_cloud_pcd(point_cloud_msg, filename):
    # Create an empty Open3D point cloud
    open3d_cloud = o3d.geometry.PointCloud()

    # Convert the ROS point cloud to Open3D format
    points = list(pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z")))
    open3d_cloud.points = o3d.utility.Vector3dVector(points)

    # Save Open3D point cloud to .pcd file
    o3d.io.write_point_cloud(filename, open3d_cloud)

    # print(f"Point cloud saved as {filename}")



def odometry_callback(msg):
    global last_yaw, pub, assemble_scans, start_time, map_count
    global prev_yaw, direction, last_change_time, cycle_count, start_time 
    global yaw_angles, roll_angles, pitch_angles, timestamps, prev_yaw, crossed_zero_pos, crossed_zero_neg, cycle_count
    global resp  # Declare resp as global

    # Extract yaw from odometry message
    orientation = msg.pose.pose.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.euler_from_quaternion(quaternion)
    yaw = math.degrees(euler[2])  # yaw in radians
    pitch = math.degrees(euler[1])  # yaw in radians
    roll = math.degrees(euler[0])  # yaw in radians
    # print("Yaw: {:.1f} ".format(yaw) )

    # Append the yaw angle and current time to the lists
    yaw_angles.append(yaw)
    pitch_angles.append(pitch)
    # roll_angles.append(roll)
    # timestamps.append(rospy.Time.now().to_sec())

 # Initialize start_time on first callback
    if start_time is None:
        start_time = rospy.Time.now()

    try:
        end_time = rospy.Time.now()
        resp = assemble_scans(start_time, end_time)
        pub.publish(resp.cloud)
        print ("Got cloud with %u points" % len(resp.cloud.data))

    except ServiceException as e:
        print(f"ServiceException during scan assembly: {e}")
        return

    # Check if enough time has passed since the last direction change
    if last_change_time is not None:
        time_since_last_change = rospy.get_time() - last_change_time
        if time_since_last_change < TIME_THRESHOLD and (abs(yaw) < 10 or abs(yaw) > 150) :
            print("Ignoring sudden direction change at yaw: ", round(yaw))
            # print("time_since_last_change: ", time_since_last_change, new_direction, direction )
            prev_yaw = yaw
            return


 # Check if enough time has passed since the last direction change
    if prev_yaw is not None:
        yaw_diff = yaw - prev_yaw
        # Normalize yaw difference to handle wrap-around
        # if yaw_diff > math.pi:
        #     yaw_diff -= 2 * math.pi
        # elif yaw_diff < -math.pi:
        #     yaw_diff += 2 * math.pi

        # Detect zero crossing with hysteresis and threshold
        if prev_yaw < 0 and yaw >= -ZERO_THRESHOLD:
            crossed_zero_pos = True
            last_change_time = rospy.get_time()
            print("crossed_zero_pos,  prev_yaw", prev_yaw, "yaw", yaw, "abs(yaw_diff)", (yaw_diff))
            print(f"============================================================== Completed a 180-degree cycle: {cycle_count}")

        if prev_yaw > 0 and yaw <= ZERO_THRESHOLD:
            crossed_zero_neg = True
            last_change_time = rospy.get_time()
            print("crossed_zero_neg,  prev_yaw", prev_yaw, "yaw", yaw, "abs(yaw_diff)", (yaw_diff))
            print(f"============================================================== Completed a 180-degree cycle: {cycle_count}")



    print("yaw: ", round(yaw), "zero_pos->",crossed_zero_pos, " zero_neg->",crossed_zero_neg, "count", cycle_count)


 # Check for full 360-degree cycle
    if crossed_zero_pos and crossed_zero_neg:
        cycle_count += 1
        print(f"============================================================== Completed a 360-degree cycle: {cycle_count}")
        crossed_zero_pos = False
        crossed_zero_neg = False
        start_time = end_time
        print("Start T", start_time.to_sec(), " End T: ", end_time.to_sec())
        # Save the assembled cloud as a .pcd file
        filename = f'capture000{map_count:02}.pcd'
        save_point_cloud_pcd(resp.cloud, filename)
        # Save point cloud to .pcd file
        map_count += 1


    prev_yaw = yaw




def main():
    global last_yaw, pub, assemble_scans, start_time, map_count
    map_count =1

    rospy.init_node("assemble_scans_to_cloud_line")

    # Initialize variables
    last_yaw = None
    pub = rospy.Publisher("/laser_pointcloud_assembler_line", PointCloud2, queue_size=1)
    rospy.Subscriber("/vins_estimator/odometry", Odometry, odometry_callback)
    
    # Wait for the assemble_scans2 service
    rospy.wait_for_service("assemble_scans2_line")
    assemble_scans = rospy.ServiceProxy('assemble_scans2_line', AssembleScans2)

    # rospy.wait_for_service("assemble_scans2")
    # assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)


    # Plotting function
    def plot_yaw(i):
        plt.cla()
        plt.plot(yaw_angles, label='Yaw Angle')
        plt.plot(pitch_angles, label='Pitch Angle')
        # plt.plot(roll_angles, label='Roll Angle')
        plt.xlabel('Time (s)')
        plt.ylabel('(Degree)')
        plt.title('Crane Boom Yaw and Pitch Angle Over Time')
        plt.legend()
        plt.grid(True)

    # Set up the figure and animation
    fig = plt.figure()
    ani = FuncAnimation(fig, plot_yaw, interval=100)

    # Show the plot
    plt.show()


    rospy.spin()

    atexit.register(save_final_map)


if __name__ == "__main__":
    main()
