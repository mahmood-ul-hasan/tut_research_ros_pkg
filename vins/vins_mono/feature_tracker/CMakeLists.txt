cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    vision_msgs
    message_generation
    message_runtime
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  YoloResult.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  vision_msgs
)

catkin_package(
  CATKIN_DEPENDS
  vision_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/devel/include  # Add this to make sure generated headers are included

)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
)

target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
