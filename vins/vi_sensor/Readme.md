Steps to calubrate

# ros camera calibrator optor
1. rosrun optor_stereo_visensor_ros stereo_visensor_node
2. rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.03 --no-service-check --approximate=0.1 right:=/camera/right/image_raw_4hz left:=/camera/left/image_raw_4hz right_camera:=/camera/right left_camera:=/camera/left
3. rosrun camera_calibration cameracheck.py  monocular:=/camera/left image:=/camera/left --size 7x5


# ros imu calibration using imu_util

# kalibr
# ------------------------
# It is better for mono use only one camera
# ------------------------------------
1. roslaunch optor_stereo_visensor_ros optor_stereo_visensor_ros.launch

2. reduce the hz. It is recommended to lower the frequency of the camera streams to around 4 Hz while capturing the calibration data.

rosrun topic_tools throttle messages /camera/left/image_raw 4.0 /camera/left/image_raw_4hz
rosrun topic_tools throttle messages /camera/right/image_raw 4.0 /camera/right/image_raw_4hz

3. record the camera data
rosbag record  /camera/left/image_raw_4hz  /camera/right/image_raw_4hz -o optor_static_camera_calib_data_23_2_8.bag

4. source ~/kalibr_workspace/devel/setup.bash 

5. rosrun kalibr kalibr_calibrate_cameras --bag optor_static_camera_calib_data_23-02-07.bag --topics /camera/right/image_raw_4hz /camera/left/image_raw_4hz --models pinhole-radtan pinhole-radtan --target target_checkerboard.yaml --approx-sync 0.05



6. kalibr_calibrate_imu_camera --target target_checkerboard.yaml  --cam optor_static_camera_calib_data_23_2_7-camchain.yaml --imu optor_imu_kalibr.yaml --bag optor_joint_imu_cam_calib_data_23_2_8.bag --bag-from-to 5 45



7. to convert to rovio file  kalibr_rovio_config –cam camchain.yaml
rosrun kalibr kalibr_rovio_config --cam optor_joint_imu_cam_calib_data_23_2_8-camchain-imucam.yaml 


# ===================================================
# ros camera mcm
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.03 image:=/usb_cam_mcm/image_raw_4hz camera:=/usb_cam_mcm
# Kalibar
rosrun kalibr kalibr_calibrate_cameras  --bag kalibr_satic_camera_calibration.bag --topics /usb_cam_mcm/image_raw_4hz --models pinhole-radtan --target april_6x6_80x80cm

rosrun kalibr kalibr_calibrate_imu_camera --target april_6x6_80x80cm.yaml --cam kalibr_satic_camera_calibration-camchain.yaml --imu kalibr_xsens_imu.yaml --bag rotation_displace_focus_to_target_joint_imu_cam_calib.bag --bag-from-to 10 156.300040
 # to convert to rovio file  kalibr_rovio_config –cam camchain.yaml
rosrun kalibr kalibr_rovio_config --cam rotation_displace_focus_to_target_joint_imu_cam_calib-camchain-imucam.yaml 

ROS_NAMESPACE=usb_cam_mcm rosrun image_proc image_proc





# ===================================================
# Steps to find the camera calibration error
Linearity RMS error is a measure of the deviation from linearity of the image sensor's response to incoming light. A low linearity RMS error indicates that the image sensor's response is close to being perfectly linear, which is important for accurate color reproduction and brightness levels.

Reprojection RMS error, on the other hand, measures the deviation between the observed and predicted image points after applying the camera model. A low reprojection RMS error indicates that the camera model is accurate and the predicted image points are close to the observed ones.
The reprojection error is the total sum of squared distances between the observed feature points and the projected object points

In general, values of linearity RMS error and reprojection RMS error below 1% are considered good, while values below 0.5% are considered excellent. 
# =====================================================
1. rosbag play 2023-02-12-22-55-12.bag
2. python3 cam_yaml_to_cam_info_publisher.py   # convert yaml to rostopic
3. ROS_NAMESPACE=camera/left rosrun image_proc image_proc #create rectified img
4. rosrun camera_calibration cameracheck.py  monocular:=/camera/left image:=/camera/left/image_raw --size 7x5 --square 0.03 







# Some confussion needed to clerify
# what is image_proc
ROS_NAMESPACE=camera/left rosrun image_proc image_proc
