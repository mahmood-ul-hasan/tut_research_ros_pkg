#!/usr/bin/env python

import subprocess
import time

file_paths = [
    "/home/aisl2/catkin_ws/src/vins/tf2_broadcaster.py",
    "/home/aisl2/catkin_ws/src/vins/odom_to_path_gt.py",
    "/home/aisl2/catkin_ws/src/vins/odom_to_path.py",
    "/home/aisl2/catkin_ws/src/vins/odom_republisher_in_world_frame.py",
    "/home/aisl2/catkin_ws/src/vins/odom_aligned_republisher.py"
]

launch_command = "roslaunch robot_localization ekf_real_sense.launch"

# Execute the launch command
launch_process = subprocess.Popen(["roslaunch", "robot_localization", "ekf_real_sense.launch"])

# Wait for a few seconds for the launch command to initialize
time.sleep(5)

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
