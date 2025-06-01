#!/usr/bin/env python

import subprocess
import time

file_paths = [

    # crane structural info method
    "/home/aisl2/catkin_ws/src/latest_crane/laser_to_pcl/src/tf2_broadcaster_by_imu_crane_structure.py",
    "/home/aisl2/catkin_ws/src/latest_crane/laser_to_pcl/src/combine_tf_frames_and_publish_odom_of_crane_structural.py",

    # vins method
    "/home/aisl2/catkin_ws/src/vins/odom_real_sense_republisher_in_world_frame.py",
    "/home/aisl2/catkin_ws/src/vins/odom_vins_republisher_in_world_frame.py"


]


launch_process = subprocess.Popen(["roslaunch", "tf_to_trajectory", "vins_odom_to_path.launch"])
time.sleep(5)  # Wait for a few seconds for the launch command to initialize

# Execute the launch command
# launch_process = subprocess.Popen(["roslaunch", "robot_localization", "ekf_real_sense.launch"])
# time.sleep(5) # Wait for a few seconds for the launch command to initialize


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
