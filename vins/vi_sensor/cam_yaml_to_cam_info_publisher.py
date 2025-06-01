
#!/usr/bin/env python3
# license removed for brevity


# pointgrey_camera_driver (at least the version installed with apt-get) doesn't
# properly handle camera info in indigo.
# This node is a work-around that will read in a camera calibration .yaml
# file (as created by the cameracalibrator.py in the camera_calibration pkg),
# convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
# topic.
# The yaml parsing is courtesy ROS-user Stephan:
#     http://answers.ros.org/question/33929/camera-calibration-parser-in-python/
# This file just extends that parser into a rosnode.
# """
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


def yaml_to_CameraInfo_with_timestamp(msg):
    camera_info_msg.header.frame_id = msg.header.frame_id
    camera_info_msg.header.stamp = msg.header.stamp
    # Run publisher
    # publisher = rospy.Publisher("/camera/left/camera_info", CameraInfo, queue_size=10)
    publisher = rospy.Publisher("/usb_cam_mcm/camera_info", CameraInfo, queue_size=10)

    publisher.publish(camera_info_msg)
    print(camera_info_msg)


def yaml_to_CameraInfo(yaml_fname):
    # """
    # Parse a yaml file containing camera calibration data (as produced by
    # rosrun camera_calibration cameracalibrator.py) into a
    # sensor_msgs/CameraInfo msg.

    # Parameters
    # ----------
    # yaml_fname : str
    #     Path to yaml file containing camera calibration data
    # Returns
    # -------
    # camera_info_msg : sensor_msgs.msg.CameraInfo
    #     A sensor_msgs.msg.CameraInfo message containing the camera calibration
    #     data
    # """
    # Load data from file

    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    print(camera_info_msg)
    return camera_info_msg


if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    # import argparse
    # arg_parser = argparse.ArgumentParser()
    # arg_parser.add_argument("filename", help="Path to yaml file containing " +\
    #                                          "camera calibration data")
    # args = arg_parser.parse_args()
    # filename = args.filename

    rospy.init_node("camera_info_publisher", anonymous=True)

    # Parse yaml file
    # filename = "/home/aisl/catkin_ws/src/optor_stereo_visensor/calibration/calibrationdata_23_2_8_left_cam.yaml"
    filename = "/home/aisl/catkin_ws/src/vi_sensor/mcm/mcm_320_calibration_ros.yml"
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    # image_sub = rospy.Subscriber("/camera/left/image_raw", Image, yaml_to_CameraInfo_with_timestamp)
    image_sub = rospy.Subscriber("/usb_cam_mcm/image_raw", Image, yaml_to_CameraInfo_with_timestamp)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

    # while not rospy.is_shutdown():

    #     rate.sleep()
