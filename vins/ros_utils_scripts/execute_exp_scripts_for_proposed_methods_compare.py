#!/usr/bin/env python

import subprocess
import time

# The first program (tf2_broadcaster_by_imu_crane_structure) subscribes to IMU data to broadcast the transformation (TF) along with the structural information of a crane.
#  It also re-initializes the rotation to zero, ensuring a clean start.

# The second program (combine_tf_frames_and_publish_odom_of_crane_structural) combines intermediate ROS-TF data and publishes
# the odometry of the rotating base. It reinitializes the position to zero, aligning it with the ground truth for a synchronized start.

# The third program (gt_odom_republisher_in_world_frame) transfers the ground truth pose into the world frame,
# ensuring accurate representation in the global reference frame.


# The fourth program (tf_to_trajectory_crane_structure) generates ROS path topics for both estimated pose and ground truth pose,
# enabling visualization and analysis of crane movement over time.

file_paths = [
    "/home/aisl2/catkin_ws/src/vins/tf2_broadcaster_by_imu_crane_structure_using_quat.py",
    "/home/aisl2/catkin_ws/src/vins/odom_crane_structural_info_republisher_in_world_frame_using_quat.py",
    "/home/aisl2/catkin_ws/src/vins/odom_motion_cap_sys_republisher_in_world_frame_using_quat.py",
    "/home/aisl2/catkin_ws/src/vins/odom_real_sense_republisher_in_world_frame_using_quat.py",
    "/home/aisl2/catkin_ws/src/vins/odom_vins_republisher_in_world_frame_using_quat.py"

    # "/home/aisl2/catkin_ws/src/data/model_crane_data/23_10_13_model_crane_data_to_comapre_different_methods_with_ground_truth/odometry_publisher_from_csv_file.py"


]


# Execute the launch command
launch_process = subprocess.Popen(["roslaunch", "tf_to_trajectory", "vins_odom_to_path.launch"])

# Wait for a few seconds for the launch command to initialize
time.sleep(3)

# Execute the Python scripts
processes = []
for file_path in file_paths:
    process = subprocess.Popen(["python3", file_path])
    print("running", file_path)
    processes.append(process)

    # Wait for 0.5 seconds between each execution
    time.sleep(0.5)

# Wait for user input to terminate the nodes
input("Press enter to terminate the nodes...\n")

# Terminate the launch process
launch_process.terminate()

# Terminate the Python script processes
for process in processes:
    process.terminate()
