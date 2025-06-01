0) Prerequisites
Ubuntu 20.04
ROS noetic


1) Install dependency packages

sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping ros-noetic-octomap-msgs ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server
sudo apt install ros-noetic-rqt-joint-trajectory-controller
sudo apt install libsuitesparse-dev libqglviewer-dev-qt4 ros-noetic-libg2o
$ sudo ln -s /usr/lib/x86_64-linux-gnu/libQGLViewer-qt4.so /usr/lib/x86_64-linux-gnu/libQGLViewer.so

sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers
sudo apt install ros-noetic-rqt-joint-trajectory-controller
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt-get install ros*controller*

sudo apt-get install ros-noetic-nmea-msgs 



2) step to compile by ctkin_make

rosdep install --from-paths . --ignore-src -y
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y

catkin_make --pkg orion_rotating_base
catkin_make --pkg k_crane_gazebo
catkin_make --pkg pointcloud_integrator
catkin_make --pkg model_msg_to_rotbase_joint_msg
catkin_make --pkg ridar_scan

==============================================================================================
To use the k_crane package, you need the fanda and Gpop packages (sorry for forgetting the instructions).
Clone and compile the required packages using the following command:
$ cd
$ mkdir crane_simulator
$ cd crane_simulator
$ git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeKawasaki/fanda.git
$ cd fanda
$ mkdir build
$ cd build/
$ cmake ../
$ make
$ sudo make install
$ cd ~/crane_simulator
$ git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeKawasaki/gpop.git
$ cd Gpop
$ mkdir build
$ cd build/
$ cmake ../
$ make
$ sudo make install
===========================================================================================

catkin_make --pkg code_utils

catkin_make --pkg k_crane_control
catkin_make



============================================
3) After cakin_make sucess
=============================================


Please check catkin_ws/src/k_crane/docs/site_English/index.html and go through the tutorial. (The Description tutorial cannot be executed well at this time, so please execute the Gazebo tutorial.)
Please use the "rqt" command to change the pitch and yaw of the crane.



show crane in rviz
  No hanging load  Start rviz and display the crane.
     roslaunch k_crane_description k_crane_display.launch 
  With hanging load
      roslaunch k_crane_description k_crane_with_payload_display.launch 

Gazebo
  A bare simulator with no sensors installed
     roslaunch k_crane_gazebo k_crane_gazebo.launch 
Simulator equipped with camera, suspended load, and sensor
     roslaunch k_crane_gazebo k_crane_with_payload.launch 

With gazebo started, start rqt to start the control board.

rqt
