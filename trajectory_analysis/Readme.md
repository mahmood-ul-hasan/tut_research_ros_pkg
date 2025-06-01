Objective of code is to compare different trajectries estimated by differernt methods using sensor attached to model crane


./odometry_publisher_from_csv_file.py  /path/to/your/csv/file.csv
odometry_publisher_from_csv_file.py is a Python script designed to convert pose data stored in a CSV file into ROS (Robot Operating System) odometry messages. The script reads the CSV file containing timestamped pose information, typically representing the position and orientation of a robot in 3D space. It then publishes this information as ROS odometry messages to the appropriate topic. This tool is useful for simulating or replaying robot motion trajectories stored in a CSV format within a ROS environment, facilitating testing, analysis, or visualization of robot navigation and localization algorithms.


 The first program (tf2_broadcaster_by_imu_crane_structure) subscribes to IMU data to broadcast the transformation (TF) along with the structural information of a crane.
  It also re-initializes the rotation to zero, ensuring a clean start.

The second program (combine_tf_frames_and_publish_odom_of_crane_structural) combines intermediate ROS-TF data and publishes
 the odometry of the rotating base. It reinitializes the position to zero, aligning it with the ground truth for a synchronized start.

 The third program (gt_odom_republisher_in_world_frame) transfers the ground truth pose into the world frame,
 ensuring accurate representation in the global reference frame.


 The fourth program (tf_to_trajectory_crane_structure) generates ROS path topics for both estimated pose and ground truth pose,
 enabling visualization and analysis of crane movement over time.


### some random commands 
- $ rosrun tf2_tools view_frames.py 
- $ evince frames.pdf 
- $ rosrun tf static_transform_publisher 0 0 0 -1.5708 0 0 world camera_odom_frame 0.0001
- $ rosbag play 23_6_13_boom_only_horizontal_motion.bag --clock  /tf:=/tf_dev_null  /tf_static:=/tf_static_null

#
__________
______
===========================================================
# Compare different proposed VINS methods Using Model Crane
============================================================
##  Steps to run data analysis
--------------------------------------
1. turn on simulated time
    - $ roscore
    - $ rosparam set use_sim_time true
2. run scripts
    - roslaunch trajectory_analysis run_analysis_scripts.launch 
        - it launch the file  which run different scrips
4. play rosbag
    - cd ~/catkin_ws/src/data/model_crane_data/23_9_21_model_crane_data_to_compare_different_methods
    -  rosbag play random_2.bag /tf:=/tf_dev_null  /tf_static:=/tf_static_null --clock
5. To record data for ov_avel
    - roslaunch ov_eval record_data_23_10_13_model_crane.launch 
6. To visulise the plot
    - cd ~/catkin_ws/src/data/model_crane_data/23_9_21_model_crane_data_to_compare_different_methods
    - ./3d_trajectory_plot.py 
    - ./execute_plot_scripts.py 


==============================================================================
# Create a trajectory using crane structure based method or VINS based Method
==============================================================================
##  Steps to run data analysis
1. turn on simulated time
    - $ roscore
    - $ rosparam set use_sim_time true
2. python3 tf2_broadcaster_by_imu_crane_structure_using_quat_kobelco_2022.py  
3. python3 odom_crane_structural_info_republisher_in_world_frame_using_quat.py 

4. roslaunch ultralytics_ros tracker.launch 
5. roslaunch vins_estimator crane_realsense_fisheye_t265_kobelco.launch 
6. python3 odom_vins_republisher_in_world_frame_using_quat.py 

4. roslaunch tf_to_trajectory vins_odom_to_path.launch 

5. rosbag play 5_varying\ boom\ Angle\ during\ cycle.bag --clock  /tf:=/tf_dev_null  /tf_static:=/tf_static_null -r 3



==============================================================================
# Build 3d map using crane structure based method or VINS based Method
==============================================================================
./LaserAssembler_exp_real_crane_bag.py 
roslaunch laser_to_pcl laser_scan_start_exp_real_crane_bag.launch 

build map using crane_info
python3 tf2_broadcaster_kobelco_2022.py 

build map using vins
python3 tf2_broadcaster_kobelco_2022_real_sense.py 

rosbag play 1_varying_speed.bag --clock /tf:=/tf_dev_null  /tf_static:=/tf_static_null -r 3