step 

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


catkin_make --pkg k_crane_control
catkin_make



============================================
After cakin_make sucess
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