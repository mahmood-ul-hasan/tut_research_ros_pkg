cmake_minimum_required(VERSION 3.10)
project(crop_box_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


find_package(PCL REQUIRED COMPONENTS common filters)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  tf2_eigen
  message_generation
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  geometry_msgs
  sensor_msgs
)

# Include directories for PCL and Catkin
include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package( ## if you dont add this, executables are not found
  LIBRARIES pcl_ros
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${TF2_INCLUDE_DIRS}
  ${TF2_ROS_INCLUDE_DIRS}
)

add_executable(apply_crop_box_filter src/apply_crop_box_filter.cpp)

target_link_libraries(apply_crop_box_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${TF2_LIBRARIES}
  ${TF2_ROS_LIBRARIES}
)
