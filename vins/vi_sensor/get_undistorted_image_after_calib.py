#!/usr/bin/env python3
# license removed for brevity
from sensor_msgs.msg import Image
import cv2
import rospy
import yaml


def undistorted_img(msg):
    global camera_matrix, dist_coeffs
    img = msg.data
    # Load the calibration parameters using the cv2.FileStorage class in OpenCV. For example:
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)
# Display or save the undistorted image as needed.
    cv2.imshow('Undistorted Image', undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":

    filename = '/home/aisl/catkin_ws/src/optor_stereo_visensor/calibration/calibrationdata_23_2_5_left_cam.yaml'

    with open(filename, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
    # Parse
    width = calib_data["image_width"]
    height = calib_data["image_height"]
    camera_matrix = calib_data["camera_matrix"]
    dist_coeffs = calib_data["distortion_coefficients"]

    print(camera_matrix)

    rospy.init_node("undistorted_img_publisher", anonymous=True)

    # Initialize publisher node
    image_sub = rospy.Subscriber("/camera/left/image_raw", Image, undistorted_img)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

    # while not rospy.is_shutdown():

    #     rate.sleep()
