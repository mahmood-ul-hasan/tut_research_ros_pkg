

================================================
# Compare different VINS methods for paper "Sensor Pose Estimation and 3D Mapping for Crane Operations Using Sensors Attached to the Crane Boom"
=============================================


##  Steps to run data analysis
--------------------------------------
1. turn on simulated time
    - $ roscore
    - $ rosparam set use_sim_time true
2. tf and basic comands
    - $ ./execute_experiment_scripts.py
3. vins mono
    - $ roslaunch vins_estimator crane_realsense_fisheye_t265.launch 
3. vins_fusion 
    - rosrun vins vins_node /home/aisl2/catkin_ws/src/vins/VINS_Fusion-simulation/config/crane/crane_realsense_stero_imu_config.yaml
    - rosrun vins vins_node /home/aisl2/catkin_ws/src/vins/VINS_Fusion-simulation/config/crane/crane_realsense_mono_config.yaml
    - roslaunch vins vins_rviz.launch
4. play bag file
    - rosbag play 2023-06-20-random_07.bag --clock  /tf:=/tf_dev_null  /tf_static:=/tf_static_null -u 200
_______________
5. EKF output
    - rostopic echo /odometry/filtered 
    - rostopic echo /msf/path 
_______________
6. To record data for ov_avel
    - roslaunch ov_eval record.launch 
7. To visulise the plot
    - cd ~/catkin_ws/src/data/model_crane_data/real_sense
    - ./3d_trajectory_plot.py 
    - ./execute_plot_scripts.py 
________________
# =======================================
_______________



### othere method to execute comparasion
- roscore
- rosparam set use_sim_time true
- roslaunch robot_localization ekf_real_sense.launch 
- cd ~/catkin_ws/src/vins
- ./tf2_broadcaster.py 
- ./odom_to_path.py 
- roslaunch vins_estimator crane_realsense_fisheye_t265.launch 
- rosbag play 23_6_13_boom_only_horizontal_motion.bag --clock  /tf:=/tf_dev_null  /tf_static:=/tf_static_null


