#!/usr/bin/env python3

import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import math
import tf.transformations as tf
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import atexit
from rospy.service import ServiceException

# Global variables for the first laser assembler
resp = None
prev_yaw = None
direction = None
last_change_time = None
yaw_angles = []
pitch_angles = []
roll_angles = []
timestamps = []
cycle_count = 0
crossed_zero_pos = False
crossed_zero_neg = False
start_time = None
ZERO_THRESHOLD = 6


# Time threshold to ignore sudden changes (in seconds)
TIME_THRESHOLD = 50 # time for 4
# TIME_THRESHOLD = 30 # time for 5

# Global variables for the second laser assembler
resp_line = None
map_count = 1

def save_final_map():
    global resp, map_count
    if resp and len(resp.cloud.data) > 0:
        filename = f'capture000{map_count:02}.pcd'
        save_point_cloud_pcd(resp.cloud, filename)
        print(f"Final map saved as {filename}")

def save_final_map_line():
    global resp_line, map_count_line
    if resp_line and len(resp_line.cloud.data) > 0:
        filename = f'capture_line000{map_count:02}.pcd'
        save_point_cloud_pcd(resp_line.cloud, filename)
        print(f"Final line map saved as {filename}")

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def save_point_cloud_pcd(point_cloud_msg, filename):
    # Extract and print field names
    fields = point_cloud_msg.fields
    field_names = [field.name for field in fields]
    print("Point Cloud Fields:", field_names)

    # Create an empty Open3D point cloud
    open3d_cloud = o3d.geometry.PointCloud()

    # Convert the ROS point cloud to Open3D format
    points = list(pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z", "intensity")))

    # Check if intensity field is present in the point cloud data
    if len(points[0]) >= 4:  # Assuming all tuples have at least x, y, z, intensity
        open3d_cloud.points = o3d.utility.Vector3dVector([p[:3] for p in points])  # Extract x, y, z

        # Add intensity values to Open3D point cloud
        intensities = [p[3] for p in points]
        open3d_cloud.colors = o3d.utility.Vector3dVector([[i, i, i] for i in intensities])  # Assign intensity as grayscale color
    else:
        open3d_cloud.points = o3d.utility.Vector3dVector(points)  # Fallback to using just x, y, z coordinates without intensity

    # Save Open3D point cloud to .pcd file
    o3d.io.write_point_cloud(filename, open3d_cloud)

    print(f"Point cloud saved as {filename}")


def odometry_callback(msg):
    global prev_yaw, last_change_time, cycle_count, crossed_zero_pos, crossed_zero_neg, start_time, resp, resp_line
    global assemble_scans, assemble_scans_line, pub, pub_line, map_count

    orientation = msg.pose.pose.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.euler_from_quaternion(quaternion)
    yaw = math.degrees(euler[2])
    pitch = math.degrees(euler[1])
    roll = math.degrees(euler[0])

    yaw_angles.append(yaw)
    pitch_angles.append(pitch)

    if start_time is None:
        start_time = rospy.Time.now()

    end_time = rospy.Time.now()

    try:
        resp = assemble_scans(start_time, end_time)
        pub.publish(resp.cloud)
        print ("Got cloud with %u points" % len(resp.cloud.data))

    except ServiceException as e:
        print(f"ServiceException during scan assembly: {e}")
        return
    

    try:
        resp_line = assemble_scans_line(start_time, end_time)
        pub_line.publish(resp_line.cloud)
        print ("Got line cloud with %u points" % len(resp_line.cloud.data))
    
    except ServiceException as e:
        print(f"ServiceException during scan assembly: {e}")
        return

    if last_change_time is not None:
        time_since_last_change = rospy.get_time() - last_change_time
        if time_since_last_change < TIME_THRESHOLD and (abs(yaw) < 10 or abs(yaw) > 150):
            print("Ignoring sudden direction change at yaw: ", round(yaw))
            prev_yaw = yaw
            return

    if prev_yaw is not None:
        yaw_diff = yaw - prev_yaw
        if prev_yaw < 0 and yaw >= -ZERO_THRESHOLD:
            crossed_zero_pos = True
            last_change_time = rospy.get_time()
            print("crossed_zero_pos, prev_yaw", prev_yaw, "yaw", yaw, "abs(yaw_diff)", (yaw_diff))
            print(f"============================================================== Completed a 180-degree cycle: {cycle_count}")

        if prev_yaw > 0 and yaw <= ZERO_THRESHOLD:
            crossed_zero_neg = True
            last_change_time = rospy.get_time()
            print("crossed_zero_neg, prev_yaw", prev_yaw, "yaw", yaw, "abs(yaw_diff)", (yaw_diff))
            print(f"============================================================== Completed a 180-degree cycle: {cycle_count}")

    print("yaw: ", round(yaw), "zero_pos->", crossed_zero_pos, " zero_neg->", crossed_zero_neg, "count", cycle_count)

    if crossed_zero_pos and crossed_zero_neg:
        cycle_count += 1
        print(f"============================================================== Completed a 360-degree cycle: {cycle_count}")
        crossed_zero_pos = False
        crossed_zero_neg = False
        start_time = end_time

        print("Start T", start_time.to_sec(), " End T: ", end_time.to_sec())
        filename = f'capture000{map_count:02}.pcd'
        save_point_cloud_pcd(resp.cloud, filename)


        filename_line = f'capture_line000{map_count:02}.pcd'
        save_point_cloud_pcd(resp_line.cloud, filename_line)
        print(f"Final line map saved as {filename}")

        save_point_cloud_pcd(resp.cloud, filename)
        map_count += 1

    prev_yaw = yaw

def main():
    global assemble_scans, assemble_scans_line, pub, pub_line

    rospy.init_node("assemble_scans_to_cloud2")

    pub = rospy.Publisher("/laser_pointcloud_assembler", PointCloud2, queue_size=1)
    pub_line = rospy.Publisher("/laser_pointcloud_assembler_line", PointCloud2, queue_size=1)

    rospy.Subscriber("/vins_estimator/odometry", Odometry, odometry_callback)

    rospy.wait_for_service("laser_assembler_2/assemble_scans2")
    assemble_scans = rospy.ServiceProxy('laser_assembler_2/assemble_scans2', AssembleScans2)

    rospy.wait_for_service("laser_assembler_1/assemble_scans2")
    assemble_scans_line = rospy.ServiceProxy('laser_assembler_1/assemble_scans2', AssembleScans2)

    def plot_yaw(i):
        plt.cla()
        plt.plot(yaw_angles, label='Yaw Angle')
        plt.plot(pitch_angles, label='Pitch Angle')
        plt.xlabel('Time (s)')
        plt.ylabel('(Degree)')
        plt.title('Crane Boom Yaw and Pitch Angle Over Time')
        plt.legend()
        plt.grid(True)

    fig = plt.figure()
    ani = FuncAnimation(fig, plot_yaw, interval=100)
    plt.show()

    rospy.spin()

    atexit.register(save_final_map)
    atexit.register(save_final_map_line)

if __name__ == "__main__":
    main()
