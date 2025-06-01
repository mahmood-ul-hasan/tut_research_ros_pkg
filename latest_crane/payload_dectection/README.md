an overview of payload dectection is given in ppt
payload guide.ppt

Two way to run
Save pointcloud map in pcd file formate
$ rosrun pcl_ros pointcloud_to_pcd input:=/laser_pointcloud_optimze

publish pointcloud map from pcd file using
$ rosrun pcl_ros pcd_to_pointcloud payload.pcd 5

run 
roslaunch payload_dectection payload_dectection_segmentation.launch 
