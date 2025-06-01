steps to assemble the laser scan and convert into point cloud data using laser_asembler package


$ cd src/crane/laser_to_pcl
$ rosbag play laser2.bag
$ roslauch laser_to_pcl start2.launch

Result:

process[my_assembler-1]: started with pid [15835]
process[laser2pc-2]: started with pid [15836]
Got cloud with 0 points
Got cloud with 144380 points
Got cloud with 283280 points
Got cloud with 424900 points
Got cloud with 576900 points
Got cloud with 720920 points
Got cloud with 863440 points
Got cloud with 1013820 points



Commands
rosbag info laser2.bag
rosbag play --clock bagfiles/b.bag /tilt_scan:=/scan