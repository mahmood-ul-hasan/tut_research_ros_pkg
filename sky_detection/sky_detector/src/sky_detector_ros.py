#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
import cv2
from scipy.signal import medfilt
from scipy import ndimage
import numpy as np
from matplotlib import pyplot as plt


def cal_skyline(mask):
    h, w = mask.shape
    for i in range(w):
        raw = mask[:, i]
        after_median = medfilt(raw, 19)
        try:
            first_zero_index = np.where(after_median == 0)[0][0]
            first_one_index = np.where(after_median == 1)[0][0]
            if first_zero_index > 20:
                mask[first_one_index:first_zero_index, i] = 1
                mask[first_zero_index:, i] = 0
                mask[:first_one_index, i] = 0
        except:
            continue
    return mask


def get_sky_region_gradient(img):

    h, w, _ = img.shape

    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    img_gray = cv2.blur(img_gray, (9, 3))
    cv2.medianBlur(img_gray, 5)
    lap = cv2.Laplacian(img_gray, cv2.CV_8U)
    gradient_mask = (lap < 6).astype(np.uint8)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 3))

    mask = cv2.morphologyEx(gradient_mask, cv2.MORPH_ERODE, kernel)
    # plt.imshow(mask)
    # plt.show()
    mask = cal_skyline(mask)
    after_img = cv2.bitwise_and(img, img, mask=mask)

    return after_img



def image_callback(data):
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # Process the image
        img_sky = get_sky_region_gradient(cv_image)
        # Convert the processed image back to ROS Image message
        img_sky_rosmsg = bridge.cv2_to_imgmsg(img_sky, encoding="bgr8")
        # Publish the processed image
        result_publisher.publish(img_sky_rosmsg)
        # Show the processed image (optional)
        # plt.imshow(img_sky, cmap='gray')
        # plt.title('Processed Image')
        # plt.axis('off')
        # plt.show(block=False)  # Display the image without blocking
    except Exception as e:
        rospy.logerr("Error processing image: {}".format(e))



if __name__ == '__main__':

    rospy.init_node('sky_detector_ros_py', anonymous=True)
    
    bridge = CvBridge()
    # Define the publisher for the processed image
    result_publisher = rospy.Publisher('/processed_image', Image, queue_size=10)
    # Subscribe to the image topic
    rospy.Subscriber('/camera/fisheye1/image_raw', Image, image_callback)
    # Spin
    rospy.spin()
