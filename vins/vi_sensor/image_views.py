#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


def image_callback1(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image1", cv_image)
    cv2.waitKey(50)
    print(msg.header)


def image_callback2(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image2", cv_image)
    cv2.waitKey(10)
    print(msg.header)


if __name__ == '__main__':
    rospy.init_node("image_view1111")
    image_sub1 = rospy.Subscriber("/cam0/image_raw", Image, image_callback1)
    # image_sub2 = rospy.Subscriber("/camera/left/image_raw", Image, image_callback2)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
