#!/usr/bin/env python3

import rospy
import rosbag

input_bag = '/media/aisl2/aisl_data/catkin_ws/src/loam/loam_continuous/robot_city_bridge.bag'
output_bag = '/media/aisl2/aisl_data/catkin_ws/src/loam/loam_continuous/robot_city_bridge_modified.bag'
topic_name = '/sync_scan_cloud_filtered'

with rosbag.Bag(output_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if topic == topic_name and hasattr(msg, 'header') and msg.header.frame_id == "/camera":
            msg.header.frame_id = "camera"
        outbag.write(topic, msg, t)
