visualize the .pcd file in rviz
 1) rosrun pcl_ros pcd_to_pointcloud test_2021-2-20_20-1-39.pcd



visualize the .pcd file in pcd viewer
pcl_viewer test_2021-2-20_18-26-5.pcd



scan the lidar
0)  roslaunch k_crane_gazebo  k_crane_gazebo_with_imu_LIDAR.launch
1)  roslaunch ridar_scan simulation_scan_with_imu_lidar.launch
2)  rosservice call /live_scan/execute_live_scan "file_name: 'test'
speed: 1"


conversion
pcl_convert_pcd_ascii_binary <file_in.pcd> <file_out.pcd> 0
