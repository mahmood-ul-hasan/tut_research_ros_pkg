#!/bin/bash

source ~/catkin_ws/devel/setup.bash

# Execute Python script 
# python3 /path/to/your_script.py
# Execute roslaunch file
# roslaunch another_package another_launch_file.launch

python3 ~/catkin_ws/src/vins/tf2_broadcaster.py 

# sleep 0.1 # Wait for 0.5 seconds

python3 ~/catkin_ws/src/vins/odom_to_path_gt.py 

# sleep 0.1 # Wait for 0.5 seconds

python3 ~/catkin_ws/src/vins/odom_to_path.py 

# sleep 0.1 # Wait for 0.5 seconds

python3 ~/catkin_ws/src/vins/odom_republisher_in_world_frame.py 

# sleep 0.1 # Wait for 0.5 seconds

# roslaunch robot_localization ekf_real_sense.launch




