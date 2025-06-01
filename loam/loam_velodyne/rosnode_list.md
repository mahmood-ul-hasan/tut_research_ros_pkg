/laserMapping
/laserMapping_bf
/laserOdometry
/play_1651560613827884096
/rosout
/rviz
/scanRegistration
/transformMaintenance


Node [/laserOdometry]
Publications: 
 * /laser_cloud_corner_last [sensor_msgs/PointCloud2]
 * /laser_cloud_surf_last [sensor_msgs/PointCloud2]
 * /laser_odom_to_init [nav_msgs/Odometry]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /velodyne_cloud_3 [sensor_msgs/PointCloud2]

Subscriptions: 
 * /imu_trans [sensor_msgs/PointCloud2]
 * /laser_cloud_flat [sensor_msgs/PointCloud2]
 * /laser_cloud_less_flat [sensor_msgs/PointCloud2]
 * /laser_cloud_less_sharp [sensor_msgs/PointCloud2]
 * /laser_cloud_sharp [sensor_msgs/PointCloud2]
 * /velodyne_cloud_2 [sensor_msgs/PointCloud2]

Services: 
 * /laserOdometry/get_loggers
 * /laserOdometry/set_logger_level


contacting node http://Z370M-S01:35683/ ...
Pid: 418320
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (51969 - 127.0.0.1:45116) [10]
    * transport: TCPROS
 * topic: /laser_cloud_corner_last
    * to: /laserMapping
    * direction: outbound (51969 - 127.0.0.1:45088) [26]
    * transport: TCPROS
 * topic: /laser_cloud_surf_last
    * to: /laserMapping
    * direction: outbound (51969 - 127.0.0.1:45096) [28]
    * transport: TCPROS
 * topic: /velodyne_cloud_3
    * to: /laserMapping
    * direction: outbound (51969 - 127.0.0.1:45084) [24]
    * transport: TCPROS
 * topic: /laser_odom_to_init
    * to: /transformMaintenance
    * direction: outbound (51969 - 127.0.0.1:45082) [22]
    * transport: TCPROS
 * topic: /laser_odom_to_init
    * to: /laserMapping
    * direction: outbound (51969 - 127.0.0.1:45100) [30]
    * transport: TCPROS
 * topic: /tf
    * to: /rviz
    * direction: outbound (51969 - 127.0.0.1:45134) [12]
    * transport: TCPROS
 * topic: /laser_cloud_sharp
    * to: /scanRegistration (http://Z370M-S01:35933/)
    * direction: inbound (60532 - Z370M-S01:55277) [23]
    * transport: TCPROS
 * topic: /laser_cloud_less_sharp
    * to: /scanRegistration (http://Z370M-S01:35933/)
    * direction: inbound (60536 - Z370M-S01:55277) [25]
    * transport: TCPROS
 * topic: /laser_cloud_flat
    * to: /scanRegistration (http://Z370M-S01:35933/)
    * direction: inbound (60540 - Z370M-S01:55277) [27]
    * transport: TCPROS
 * topic: /laser_cloud_less_flat
    * to: /scanRegistration (http://Z370M-S01:35933/)
    * direction: inbound (60542 - Z370M-S01:55277) [29]
    * transport: TCPROS
 * topic: /velodyne_cloud_2
    * to: /scanRegistration (http://Z370M-S01:35933/)
    * direction: inbound (60544 - Z370M-S01:55277) [31]
    * transport: TCPROS
 * topic: /imu_trans
    * to: /scanRegistration (http://Z370M-S01:35933/)
    * direction: inbound (60546 - Z370M-S01:55277) [32]
    * transport: TCPROS

==========================================

Node [/transformMaintenance]
Publications: 
 * /integrated_to_init [nav_msgs/Odometry]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

Subscriptions: 
 * /aft_mapped_to_init [nav_msgs/Odometry]
 * /laser_odom_to_init [nav_msgs/Odometry]

Services: 
 * /transformMaintenance/get_loggers
 * /transformMaintenance/set_logger_level


contacting node http://Z370M-S01:42511/ ...
Pid: 418105
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (52579 - 127.0.0.1:47052) [12]
    * transport: TCPROS
 * topic: /integrated_to_init
    * to: /rviz
    * direction: outbound (52579 - 127.0.0.1:47080) [10]
    * transport: TCPROS
 * topic: /tf
    * to: /rviz
    * direction: outbound (52579 - 127.0.0.1:47072) [16]
    * transport: TCPROS
 * topic: /laser_odom_to_init
    * to: /laserOdometry (http://Z370M-S01:42523/)
    * direction: inbound (35008 - Z370M-S01:52477) [14]
    * transport: TCPROS
 * topic: /aft_mapped_to_init
    * to: /laserMapping (http://Z370M-S01:40619/)
    * direction: inbound (60202 - Z370M-S01:39611) [15]
    * transport: TCPROS


==============================================
Node [/scanRegistration]
Publications: 
 * /imu_trans [sensor_msgs/PointCloud2]
 * /laser_cloud_flat [sensor_msgs/PointCloud2]
 * /laser_cloud_less_flat [sensor_msgs/PointCloud2]
 * /laser_cloud_less_sharp [sensor_msgs/PointCloud2]
 * /laser_cloud_sharp [sensor_msgs/PointCloud2]
 * /rosout [rosgraph_msgs/Log]
 * /velodyne_cloud_2 [sensor_msgs/PointCloud2]

Subscriptions: 
 * /imu/data [unknown type]
 * /velodyne_points [sensor_msgs/PointCloud2]

Services: 
 * /scanRegistration/get_loggers
 * /scanRegistration/set_logger_level


contacting node http://Z370M-S01:35933/ ...
Pid: 418319
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (55277 - 127.0.0.1:60548) [12]
    * transport: TCPROS
 * topic: /velodyne_cloud_2
    * to: /laserOdometry
    * direction: outbound (55277 - 127.0.0.1:60544) [22]
    * transport: TCPROS
 * topic: /laser_cloud_sharp
    * to: /laserOdometry
    * direction: outbound (55277 - 127.0.0.1:60532) [19]
    * transport: TCPROS
 * topic: /laser_cloud_less_sharp
    * to: /laserOdometry
    * direction: outbound (55277 - 127.0.0.1:60536) [10]
    * transport: TCPROS
 * topic: /laser_cloud_flat
    * to: /laserOdometry
    * direction: outbound (55277 - 127.0.0.1:60540) [20]
    * transport: TCPROS
 * topic: /laser_cloud_less_flat
    * to: /laserOdometry
    * direction: outbound (55277 - 127.0.0.1:60542) [21]
    * transport: TCPROS
 * topic: /imu_trans
    * to: /laserOdometry
    * direction: outbound (55277 - 127.0.0.1:60546) [11]
    * transport: TCPROS
 * topic: /velodyne_points
    * to: /play_1651560864753202940 (http://Z370M-S01:37717/)
    * direction: inbound (36414 - Z370M-S01:50297) [18]
    * transport: TCPROS


========================================================
Node [/laserMapping]
Publications: 
 * /aft_mapped_to_init [nav_msgs/Odometry]
 * /laser_cloud_surround [sensor_msgs/PointCloud2]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /velodyne_cloud_registered [sensor_msgs/PointCloud2]

Subscriptions: 
 * /imu/data [unknown type]
 * /laser_cloud_corner_last [sensor_msgs/PointCloud2]
 * /laser_cloud_surf_last [sensor_msgs/PointCloud2]
 * /laser_odom_to_init [nav_msgs/Odometry]
 * /velodyne_cloud_3 [sensor_msgs/PointCloud2]

Services: 
 * /laserMapping/get_loggers
 * /laserMapping/set_logger_level


contacting node http://Z370M-S01:43823/ ...
Pid: 418578
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (39021 - 127.0.0.1:33530) [23]
    * transport: TCPROS
 * topic: /laser_cloud_surround
    * to: /rviz
    * direction: outbound (39021 - 127.0.0.1:33570) [16]
    * transport: TCPROS
 * topic: /velodyne_cloud_registered
    * to: /rviz
    * direction: outbound (39021 - 127.0.0.1:33568) [15]
    * transport: TCPROS
 * topic: /aft_mapped_to_init
    * to: /transformMaintenance
    * direction: outbound (39021 - 127.0.0.1:33524) [20]
    * transport: TCPROS
 * topic: /tf
    * to: /rviz
    * direction: outbound (39021 - 127.0.0.1:33572) [17]
    * transport: TCPROS
 * topic: /laser_cloud_corner_last
    * to: /laserOdometry (http://Z370M-S01:34589/)
    * direction: inbound (50620 - Z370M-S01:50973) [18]
    * transport: TCPROS
 * topic: /laser_cloud_surf_last
    * to: /laserOdometry (http://Z370M-S01:34589/)
    * direction: inbound (50626 - Z370M-S01:50973) [19]
    * transport: TCPROS
 * topic: /laser_odom_to_init
    * to: /laserOdometry (http://Z370M-S01:34589/)
    * direction: inbound (50640 - Z370M-S01:50973) [22]
    * transport: TCPROS
 * topic: /velodyne_cloud_3
    * to: /laserOdometry (http://Z370M-S01:34589/)
    * direction: inbound (50632 - Z370M-S01:50973) [21]
    * transport: TCPROS

============================