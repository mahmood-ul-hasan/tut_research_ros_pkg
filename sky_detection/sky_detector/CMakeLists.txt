cmake_minimum_required(VERSION 3.0.2)
project(sky_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  cv_bridge
  sensor_msgs
  std_msgs
)


find_package(OpenCV REQUIRED)

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sky_detector.cpp
# )

## Add cmake target dependencies of the library
## Declare a C++ executable

add_executable(sky_detector src/sky_detector.cpp)
target_link_libraries(sky_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(sky_detector_ros src/sky_detector_ros.cpp)
target_link_libraries(sky_detector_ros ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})