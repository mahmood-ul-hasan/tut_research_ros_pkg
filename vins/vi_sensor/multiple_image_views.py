#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def callback1(data):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Image 1", cv_image)
    cv2.waitKey(3)


def callback2(data):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Image 2", cv_image)
    cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node("image_display", anonymous=True)

    rospy.Subscriber("/camera/left/image_raw", Image, callback1)
    rospy.Subscriber("/cam0/image_raw", Image, callback2)

    rospy.spin()
