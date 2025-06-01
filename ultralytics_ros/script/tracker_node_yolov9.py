#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ultralytics_ros
# Copyright (C) 2023-2024  Alpaca-zip
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import torch
import os
import sys

yolo_path = '/media/aisl2/aisl_data/Agrist/yolov9-main/'  # Update this path to YOLOv9
sys.path.insert(0, yolo_path)
from models.common import DetectMultiBackend

import torch
from utils.torch_utils import select_device, smart_inference_mode

import cv_bridge
import numpy as np
import roslib.packages
import rospy
from sensor_msgs.msg import Image
# from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics_ros.msg import YoloResult
import cv2


class TrackerNode:
    def __init__(self):
        yolo_model = rospy.get_param("~yolo_model", "yolov9.pt")  # Update YOLO model
        self.input_topic = rospy.get_param("~input_topic", "image_raw")
        self.result_topic = rospy.get_param("~result_topic", "yolo_result")
        self.result_image_topic = rospy.get_param("~result_image_topic", "yolo_image")
        self.conf_thres = rospy.get_param("~conf_thres", 0.25)
        self.iou_thres = rospy.get_param("~iou_thres", 0.45)
        self.max_det = rospy.get_param("~max_det", 300)
        self.classes = rospy.get_param("~classes", None)
        self.tracker = rospy.get_param("~tracker", "bytetrack.yaml")
        self.device = rospy.get_param("~device", "cuda")
        self.result_conf = rospy.get_param("~result_conf", True)
        self.result_line_width = rospy.get_param("~result_line_width", None)
        self.result_font_size = rospy.get_param("~result_font_size", None)
        self.result_font = rospy.get_param("~result_font", "Arial.ttf")
        self.result_labels = rospy.get_param("~result_labels", True)
        self.result_boxes = rospy.get_param("~result_boxes", True)
        path = roslib.packages.get_pkg_dir("ultralytics_ros")
        self.device = select_device(self.device)
        self.half = False
        self.dnn = False
        self.data = "/media/aisl2/aisl_data/Agrist/yolov9-main/data/leave_detection.v1-branches_leaves_first.yolov9/data.yaml"  # Replace with the actual path to your YOLO config file

        # self.model = DetectMultiBackend(f"{path}/models/{yolo_model}", device=self.device)
        self.model = DetectMultiBackend(f"{path}/models/{yolo_model}", device=self.device, dnn=self.dnn, data=self.data, fp16=self.half)

        self.sub = rospy.Subscriber(
            self.input_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.results_pub = rospy.Publisher(self.result_topic, YoloResult, queue_size=1)
        self.result_image_pub = rospy.Publisher(
            self.result_image_topic, Image, queue_size=1
        )
        self.bridge = cv_bridge.CvBridge()
        self.use_segmentation = yolo_model.endswith("-seg.pt")
        kernel_size = 70
        self.kernel = np.ones((kernel_size, kernel_size), np.uint8)



    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_image = cv2.resize(cv_image, (832, 800))  # Ensure compatible dimensions
            img_tensor = torch.from_numpy(cv_image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
            results = self.model.forward(img_tensor.to(self.device))
        except RuntimeError as e:
            rospy.logerr(f"Error during forward pass: {e}")
            return

        if results:
            yolo_result_msg = YoloResult()
            yolo_result_image_msg = Image()
            yolo_result_msg.header = msg.header
            yolo_result_image_msg.header = msg.header
            yolo_result_msg.detections = self.create_detections_array(results)
            yolo_result_image_msg = self.create_result_image(results)
            
            if self.use_segmentation:
                yolo_result_msg.masks = self.create_segmentation_masks(results)

            self.results_pub.publish(yolo_result_msg)
            self.result_image_pub.publish(yolo_result_image_msg)



    def create_detections_array(self, results):
        detections = []
        for result in results:
            if isinstance(result, torch.Tensor):
                try:
                    conf = result[4].squeeze()  # Squeeze to remove single dimensions
                    if len(conf.shape) == 0:  # If it's already a scalar
                        detections.append((result[:4], conf.item()))
                    else:
                        detections.append((result[:4], conf.mean().item()))  # Example alternative method
                except IndexError:
                    rospy.logwarn(f"IndexError: {result} does not have enough dimensions.")
                    continue
        return detections



    def create_result_image(self, results):
        # Check if results is a tensor
        if isinstance(results, torch.Tensor):
            # Assuming results is a tensor representing images or feature maps
            # Convert tensor to NumPy array for visualization
            plotted_image = results[0].cpu().numpy()
        elif isinstance(results, list):
            # If results is a list of detections or bounding boxes
            plotted_image = results[0].plot(
                conf=self.result_conf,
                line_width=self.result_line_width,
                font_size=self.result_font_size,
                font=self.result_font,
                labels=self.result_labels,
                boxes=self.result_boxes,
            )
        else:
            rospy.logwarn(f"Unexpected results type: {type(results)}")
            return None

        # Convert plotted image to ROS message
        result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
        return result_image_msg




    def create_segmentation_masks(self, results):
        masks_msg = []
        for result in results:
            if hasattr(result, "masks") and result.masks is not None:
                for mask_tensor in result.masks.data:
                    mask_numpy = (
                        np.squeeze(mask_tensor.to("cpu").detach().numpy()).astype(np.uint8)
                        * 255
                    )
                    mask_image_msg = self.bridge.cv2_to_imgmsg(mask_numpy, encoding="mono8")
                    masks_msg.append(mask_image_msg)
        return masks_msg

if __name__ == "__main__":
    rospy.init_node("tracker_node")
    node = TrackerNode()
    rospy.spin()
