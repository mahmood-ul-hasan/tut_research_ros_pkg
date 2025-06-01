# orion_rotating_base
This is a driver for rotating base by Orion Giken.

rosservice call /orion_rotating_base/return_origin
roslaunch orion_rotating_base rotating_base.launch

roslaunch laser_to_pcl laser_scan_start_exp.launch

# Moves from 90 to 270
rosservice call /orion_rotating_base/cmd "end_pos: 90
speed: 3"

rosservice call /orion_rotating_base/cmd "end_pos: 270
speed: 3"
