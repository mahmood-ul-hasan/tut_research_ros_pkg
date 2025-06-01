# ridar_scan
This package is a ROS package for loading laser scan data (PTS, PLY) or obtaining LiveScan data using LIDAR.  


## Requirements
The following ROS packages are required:  
- [lms1xx](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeKawasaki/lms1xx)
- [orion_rotating_base](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeKawasaki/orion_rotating_base)
- [pointcloud_integrator](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/inouchi/pointcloud_integrator)
- [lidar_simulation_environment](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeKawasaki/lidar_simulation_environment)

```bash
$ cd ~/catkin_ws/src
$ git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeKawasaki/lms1xx.git
$ git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeKawasaki/orion_rotating_base.git
$ git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/inouchi/pointcloud_integrator.git
$ git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeKawasaki/lidar_simulation_environment.git

# for indigo
$ sudo apt-get install ros-indigo-libg2o ros-indigo-cv-bridge liblapack-dev libblas-dev freeglut3-dev libqglviewer-dev libsuitesparse-dev libx11-dev

# for kinetic
$ cd pointcloud_integrator
$ git checkout kinetic
$ sudo apt install libsuitesparse-dev libqglviewer-dev-qt4 ros-kinetic-libg2o
$ sudo ln -s /usr/lib/x86_64-linux-gnu/libQGLViewer-qt4.so /usr/lib/x86_64-linux-gnu/libQGLViewer.so

$ cd ~/catkin_ws && catkin_make -i
$ catkin_make -j8
```
Please try the following command if a compilation error is occurred by a sophus package.
```bash
# for indigo
$ sudo apt remove ros-indigo-sophus

# for kinetic
$ sudo apt remove ros-kinetic-sophus
```


## live_scan node

### Subscribed Topics:
- *`/scan`* ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))  
  
  Laser scans from 2D LIDAR.
- *`/orion_rotating_base/joint_states`* ([sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html))  
  
  Angles of the pan and rotation speed from Rotating Base.
- *`/front_laser_link/scan`* ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))  
  
  Laser scans from 2D LIDAR (in simulation).
- *`/model2scan_node/model_joint_states`* ([sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html))  
  
  Angles of the pan(yaw) from Rotating Base (in simulation).

### Published Topics:
- *`/live_scan/cloud`* ([sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html))  
  
  PointCloud computed by live_scan node.
- *`/ridar_scan/liveScan`* ([pointcloud_integrator/keyScanMsg](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/inouchi/pointcloud_integrator/blob/kinetic/msg/keyScanMsg.msg))  
  
  For publishing PointCloud computed by live_scan node to [pointcloud_integrator node](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/inouchi/pointcloud_integrator/tree/kinetic).

### Services:
- *`/live_scan/execute_live_scan`*  
  
  Execute LiveScan.

### Parameters:
All the configurable parameters are listed in *launch/live_scan.launch* as ros params.  
  
- *`~rotation_speed`* (int, default 10)  
  
  Rotation speed [deg/s] when executing LiveScan.
- *`~enable_tilt_compensation`* (bool, default: true)  
  
  Determines whether or no LiveScan uses a tilt calibration file when computing pointcloud.
- *`~ref_normalization`* (bool, default: false)  
  
  Determines whether or no LiveScan normalizes intensity (reflectance) to the specified range.
- *`~ref_min`* (int, default: 300)  
  
  Minimum number when normalizing intensity (reflectance).
- *`~ref_max`* (int, default: 900)  
  
  Maximum number when normalizing intensity (reflectance).
- *`~export_pcd`* (bool, default: false)  
  
  Determines whether or no LiveScan exports PointCloud as pcd file.
- *`~export_pts`* (bool, default: false)  
  
  Determines whether or no LiveScan exports PointCloud as pts file.
- *`~export_ply`* (bool, default: false)  
  
  Determines whether or no LiveScan exports PointCloud as ply file.
- *`~mesh_thresh`* (double, default: 0.3)  

  Threshold when generating ply file.
- *`~is_offline`* (bool, default: false)  

  Determines whether or no LiveScan is off-line with bag data.
- *`~is_simulation`* (bool, default: false)  

  Determines whether or no LiveScan is in simulation.


## Usage

### Tilt Calibration

Requirements
- scipy
- pyquaternion
- plotly [optional for visualization]

```bash
$ rosrun ridar_scan calibrate.py

# Feed scan & pan tilt data, then press Ctrl+C
# Calibration data will be saved to ridar_scan/data/calibration.csv
```

### LiveScan
Please execute LiveScan using rosservice after running dtw_sensor and simulation_scan node. As the following commands:
```bash
$ roslaunch lrs_orion_simulator dtw_sensor.launch
$ roslaunch ridar_scan simulation_scan.launch
$ rosservice call /live_scan/execute_live_scan "file_name: ''
speed: 0"

# file_name: It is point cloud file name. If it is set '', use default name 'LiveScan'.
# speed: It is rotation speed. If it is set 0, use param "rotation_speed" (normally from launch file).

# /live_scan/cloud will be published as PointCloud2 topic
# Also, PCD data will be saved to "ridar_scan/scans/" directory
```