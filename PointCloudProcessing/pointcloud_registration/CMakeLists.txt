cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_registration)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions
  roslib
  nav_msgs
  rospy

)

find_package(PCL REQUIRED COMPONENTS common io registration)
find_package(PythonLibs REQUIRED)


# Include directories for PCL
include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(include)

catkin_package( ## if you dont add this, executables are not found
  INCLUDE_DIRS include
  LIBRARIES pcl_ros
  CATKIN_DEPENDS geometry_msgs  pcl_msgs roscpp std_msgs
  DEPENDS system_lib
)



# Add the executable for the ROS node
add_executable(sequential_registration src/sequential_registration.cpp)


# Link the ROS and PCL libraries to the executable
target_link_libraries(sequential_registration
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)


