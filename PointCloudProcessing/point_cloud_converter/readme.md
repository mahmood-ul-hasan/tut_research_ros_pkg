# point_cloud_converter

The `point_cloud_converter` package provides a node to convert data between the old `sensor_msgs/PointCloud` and the new `sensor_msgs/PointCloud2` data formats, and vice versa.

## Usage

The package contains a single node (with the same name) that performs the conversion. The node subscribes to two ROS topics for input:

- `~points_in` for `PointCloud` messages.
- `~points2_in` for `PointCloud2` messages.

The node publishes the converted data to the following output topics:

- If the input data is received on `~points_in` as `PointCloud` messages, the output will be published on `~points2_out` as `PointCloud2` messages.
- If the input data is received on `~points2_in` as `PointCloud2` messages, the output will be published on `~points_out` as `PointCloud` messages.

## Examples

To use the `point_cloud_converter` node, you can include it in your launch file or run it directly from the command line.

### Example Launch File

```xml
<launch>
    <node name="point_cloud_converter" pkg="point_cloud_converter" type="point_cloud_converter" output="screen">
        <!-- Subscribe to PointCloud messages -->
        <remap from="~points_in" to="/input_point_cloud_topic"/>
        <!-- Publish converted PointCloud2 messages -->
        <remap from="~points2_out" to="/output_point_cloud2_topic"/>
        
        <!-- OR -->
        
        <!-- Subscribe to PointCloud2 messages -->
        <remap from="~points2_in" to="/input_point_cloud2_topic"/>
        <!-- Publish converted PointCloud messages -->
        <remap from="~points_out" to="/output_point_cloud_topic"/>
    </node>
</launch>


# Point Cloud Converter

## Running from the Command Line

### To convert PointCloud messages to PointCloud2:
```bash
rosrun point_cloud_converter point_cloud_converter _points_in:=/input_point_cloud_topic _points2_out:=/output_point_cloud2_topic
To convert PointCloud2 messages to PointCloud:
bash
Copy code
rosrun point_cloud_converter point_cloud_converter _points2_in:=/input_point_cloud2_topic _points_out:=/output_point_cloud_topic
Installation
Clone the repository into your catkin workspace and build the package:

bash
Copy code
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics-forks/point_cloud_converter.git
cd ~/catkin_ws
catkin_make
Dependencies
ROS (Robot Operating System)
sensor_msgs package
License
This project is licensed under the MIT License. See the LICENSE file for details.

Author
This package is maintained by PAL Robotics.




