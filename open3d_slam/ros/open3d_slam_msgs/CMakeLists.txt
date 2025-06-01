cmake_minimum_required(VERSION 3.5.1)
project(open3d_slam_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  message_generation
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Vertices.msg
  PolygonMesh.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveMap.srv
  SaveSubmaps.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
