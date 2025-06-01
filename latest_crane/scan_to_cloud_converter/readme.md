# Scan to Cloud Converter

## Overview

The `scan_to_cloud_converter` package converts `sensor_msgs/LaserScan` messages to `sensor_msgs/PointCloud2` messages. This package is useful for applications that require point cloud data derived from laser scans.

## Features

- Converts `sensor_msgs/LaserScan` to `sensor_msgs/PointCloud2`.
- The output cloud message has the `is_dense` attribute set to `false`.
- Ranges in the scan message which are outside the `(min_range, max_range)` are represented by `nan` values in the output cloud.

## Nodes

### scan_to_cloud_converter_node

The `scan_to_cloud_converter_node` is the main node in this package. It subscribes to `sensor_msgs/LaserScan` messages and publishes `sensor_msgs/PointCloud2` messages.

#### Subscribed Topics

- **scan** (`sensor_msgs/LaserScan`): The input laser scan messages.

#### Published Topics

- **cloud** (`sensor_msgs/PointCloud2`): The output point cloud messages.

## Installation

To install the `scan_to_cloud_converter` package, clone the repository into your ROS workspace and build it using `catkin_make` or `catkin build`.

```sh
cd ~/catkin_ws/src
git clone <repository_url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash

