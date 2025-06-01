#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics_ros.msg import YoloResult

def custom_message_callback(msg):
    for i, mask_image in enumerate(msg.masks):
        # Assuming mask_image is of type sensor_msgs/Image
        pub.publish(mask_image)

if __name__ == '__main__':
    rospy.init_node('custom_message_image_view')

    # Replace 'YourCustomMessage' with your actual custom message type
    rospy.Subscriber('/yolo_result', YoloResult, custom_message_callback)

    pub = rospy.Publisher('/yolo_mask', Image, queue_size=10)

    rospy.spin()

