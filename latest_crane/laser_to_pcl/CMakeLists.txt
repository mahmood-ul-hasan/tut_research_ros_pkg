cmake_minimum_required(VERSION 3.0.2)
project(laser_to_pcl)


find_package(catkin REQUIRED COMPONENTS
  laser_assembler
  laser_geometry
  rospy
  sensor_msgs
  std_msgs
  message_generation
  roscpp

)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Num.msg
#   Message2.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS laser_assembler laser_geometry rospy sensor_msgs pcl_conversions pcl_ros
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser2pointcloud2 src/laser2pointcloud2.cpp)

target_link_libraries(laser2pointcloud2
  ${catkin_LIBRARIES}
)


add_executable(tf2_broadcaster_kobelco src/tf2_broadcaster_kobelco_2022.cpp)
target_link_libraries(tf2_broadcaster_kobelco ${catkin_LIBRARIES})

