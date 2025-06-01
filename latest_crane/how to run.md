===================================================
Simulation
====================================================

# Method 01 (Simulation experiment by Master student)
roslaunch k_crane_gazebo k_crane_imu_lidar_simulation_scan.launch
roslaunch ridar_scan simulation_scan.launch
rosservice call /live_scan/execute_live_scan "file_name: 'test'
speed: 1"



# Method 02 (Simultion by Mahmood)
roslaunch k_crane_gazebo k_crane_imu_lidar_simulation_scan_using_rotate_laser_joint.launch
roslaunch laser_to_pcl laser_scan_start.launch
./tf2_broadcaster_by_imu_final.py   from /laser_to_pcl/src
./rotate_start.py   from /laser_to_pcl/src

rosbag record /front_laser_link/scan /imu_boom_link/data



===================================================
Simulation for payload 
====================================================
roslaunch k_crane_gazebo k_crane_imu_lidar_simulation_scan_using_rotate_laser_joint_payload.launch 
./tf2_broadcaster_by_imu_final.py   from /laser_to_pcl/src
roslaunch laser_to_pcl laser_scan_start.launch 
./rotate_start.py 
roslaunch payload_dectection payload_dectection_segmentation.launch 




===================================================
# Experiment realsense2_camera on model crane
====================================================
Run lidar, rotating_base, realsense2_camera, one by one

# lidar
 roslaunch urg_node urg_lidar.launch 
# Rotating base
roslaunch orion_rotating_base rotating_base.launch  
rosservice call /orion_rotating_base/cmd "end_pos: 90
speed: 1"
 rosservice call /orion_rotating_base/cmd "end_pos: 270
speed: 1"
# camera
roslaunch realsense2_camera demo_t265_model_crane.launch 
# Tf broadcaster
./tf2_broadcaster_by_realsense_t265_model_crane.py 
# aser scan to pointcloud and Assembler 
./LaserAssembler_realsense_t265_model_crane.py 
roslaunch laser_to_pcl laser_scan_combine_by_realsense_t265_model_crane.launch 
# record
 rosbag record  --exclude "/laser_pointcloud_optimze" --exclude "tf" -x "(.*)/compressed(.*)" -a 
 rosbag record  --exclude /laser_pointcloud_optimze --exclude "tf" -x "(.*)/compressed(.*)" -a 

rosbag record -e "/camera/(.*)" -x "(.*)/compressed(.*)"

rosbag record /camera/fisheye1/ /camera/fisheye1/image_raw /camera/fisheye2/camera_info /camera/fisheye2/image_raw /camera/gyro/imu_info /camera/accel/imu_info /camera/imu /scan /orion_rotating_base/joint_states /scan /tf /tf_static /trajectory /urg_node/parameter_descriptions /urg_node/parameter_updates



rosrun pcl_ros pointcloud_to_pcd input:=//laser_pointcloud_assembler _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src


rosparam set /use_sim_time true


===================================================
# Bag realsense2_camera on model crane
====================================================
$ roscore
$ rosparam set /use_sim_time true
$ rosrun rviz rviz 
$ open t265_model_crane.rviz from /home/aisl/catkin_ws/src/realsense-ros/realsense2_camera/rviz 
$ roslaunch laser_to_pcl laser_scan_combine_by_realsense_t265_model_crane.launch 
$ ./LaserAssembler_realsense_t265_model_crane.py 
$ ./tf2_broadcaster_by_realsense_t265_model_crane.py 





===================================================
Experiment
====================================================

# Experimant by bag data
1) roscore
2) roslaunch laser_to_pcl laser_scan_start_exp_bag.launch
3) rosparam set /use_sim_time true
4) ./tf2_broadcaster_by_imu_exp_crane3_bag.py
5) rosbag play D1.bag --clock
6) rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze




# Important comands used in experiment
rosbag record /scan /orion_rotating_base/joint_states /imu/time_ref /filter/quaternion /imu/acceleration /imu/angular_velocity /imu/mag /imu/data 

#---------------------------------------
rosrun tf2_tools view_frames.py
evince frames.pdf
grep lidar_sensor_model -R catkin_ws/src/latest_crane
rosrun tf static_transform_publisher 0 0 0 0 0 0 world base_link 100
rosservice call /gazebo/get_model_state "model_name: 'dtw_robot'
relative_entity_name: 'boom_link'"

rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze /catkin_ws/src/latest_crane/laser_to_pcl/src
rosrun pcl_ros pcd_to_pointcloud <file.pcd> [ <interval> ]
rosrun pcl_ros pcd_to_pointcloud noised_output.pcd /cloud_pcd:=clould_pcd2 __name:=clould_pcd2
rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze _prefix:=/home/aisl/catkin_ws/src/latest_crane/laser_to_pcl/src/

$ rosrun tf2_tools view_frames.py
$ evince frames.pdf


===================================================
#Experiment
====================================================
Setup the PC ethernet IP
Adress 192.168.0.1
Netmask 24

1)  Try to run IMU seprately using xsens_ros_mti_driver
roslaunch xsens_mti_driver display.launch
sudo chmod +666 /dev/ttyUSB0
2) run LIDAR seprately using urg_node
roslaunch urg_node urg_lidar.launch
 and set ip in launch as following
    <param name="ip_address" value="192.168.0.10"/>
3) run rotating base seprately using "orion_rotating_node"
 roslaunch orion_rotating_base rotating_base.launch
    sudo chmod +666 /dev/ttyUSB0

 4) roslaunch laser_to_pcl laser_scan_start_exp.launch
5) ./tf2_broadcaster_by_imu_exp.py   from /laser_to_pcl/src
6)  rosservice call /orion_rotating_base/cmd "end_pos: 270
speed: 1"

  rosservice call /orion_rotating_base/cmd "end_pos: 90
speed: 1"


===================================================
# Experiment with real crane kobelco uising bag file
====================================================
rosparam set use_sim_time true
roslaunch laser_to_pcl laser_scan_start_exp_real_crane_bag.launch 
roslaunch laser_filters angle_filter_example.launch 
rosrun laser_to_pcl tf2_broadcaster_kobelco 
./LaserAssembler_exp_real_crane_bag.py 
rosbag play low_pitch_angle_slow_speed_right_turn.bag  --clock /tf:=/tf_dev_null  /tf_static:=/tf_static_null


===================================================
# to implement the HDL graph slam 
# Interactive 3D Graph SLAM for Map Correction
====================================================
roslaunch laser_to_pcl hdl_laser_scan_start_exp_real_crane_bag.launch
roslaunch laser_filters angle_filter_example.launch 
rosrun laser_to_pcl tf2_broadcaster_kobelco
./hdl_laser2pointcloud.py 
./hdl_tf_listener_odm_publisher.py
./hdl_odm_pointcloud_publisher.py 
./hdl_time_synchronizer.py 
roslaunch odometry_saver online.launch
rosbag play low_pitch_angle_slow_speed_right_turn.bag --clock
rosrun interactive_slam odometry2graph
rosrun interactive_slam interactive_slam



===================================================
# Experiment kobelco 2023
====================================================
compare trajectory
---------------------------------------------------------
roscore
rosparam set use_sim_time true
roslaunch vins_estimator crane_realsense_fisheye_t265_kobelco.launch 
roslaunch ultralytics_ros tracker.launch 
roslaunch laser_to_pcl laser_scan_combine_kobelco.launch 
rosbag play 1_varying_speed.bag --clock /tf:=/tf_dev_null  /tf_static:=/tf_static_null -r 3
roslaunch laser_filters angle_filter_example.launch 


Build 3d map
------------------------------
./LaserAssembler_exp_real_crane_bag.py 
roslaunch laser_to_pcl laser_scan_start_exp_real_crane_bag.launch 

build map using crane_info
python3 tf2_broadcaster_kobelco_2022.py 

build map using vins
python3 tf2_broadcaster_kobelco_2022_real_sense.py 

rosbag play 1_varying_speed.bag --clock /tf:=/tf_dev_null  /tf_static:=/tf_static_null -r 3

pairwise_incremental_registration







===================================================
# Experiment kobelco 2024
====================================================
compare trajectory
---------------------------------------------------------
roscore
rosparam set use_sim_time true
roslaunch vins_estimator crane_realsense_fisheye_t265_kobelco.launch 
roslaunch ultralytics_ros tracker.launch 
roslaunch laser_assembler laser_scan_combine_kobelco.launch 
rosbag play 1_varying_speed.bag --clock /tf:=/tf_dev_null  /tf_static:=/tf_static_null -r 3
roslaunch laser_filters angle_filter_example.launch 


Build 3d map
------------------------------
./LaserAssembler_exp_real_crane_bag.py 
roslaunch laser_assembler laser_scan_start_exp_real_crane_bag.launch 

build map using crane_info
python3 tf2_broadcaster_kobelco_2022.py 

build map using vins
python3 tf2_broadcaster_kobelco_2022_real_sense.py 

rosbag play 1_varying_speed.bag --clock /tf:=/tf_dev_null  /tf_static:=/tf_static_null -r 3





===================================================
# pairwise_incremental_registration
====================================================
roslaunch laser_assembler laser_scan_combine_pairwise_incremental_registration_real_sense_kobelco_2024.launch

roscd lasser_assembler/script

LaserAssembler_pairwise_incremental_registration.py
got to cd /media/aisl2/aisl_data/code_ws/pairwise_incremental_registration_using_pcl$ 


