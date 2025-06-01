cmake_minimum_required(VERSION 2.8.3)
project(ridar_scan)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#SET(CMAKE_BUILD_TYPE Debug) # NOTE: Debug mode causes segfault because of PCL when C++11 is used.
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

#set(CMAKE_C_COMPILER "clang")
#set(CMAKE_CXX_COMPILER "clang++")

# For profiling with gprof
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/thirdparty/Sophus)

#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -Wl,--no-as-needed" ) 
#set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -std=c++11 -Wl,--no-as-needed" )

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
#SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops") #fomit-frame-pointer and -pg are incompatible
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee  -fno-signed-zeros -fno-math-errno -funroll-loops")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  image_geometry
  image_transport
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  pointcloud_integrator
  gazebo_ros 
  gazebo_msgs
  message_filters
  image_transport
)

#Libraries for depthfilter
find_package(Eigen3 REQUIRED)
#FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ExecuteLiveScan.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs gazebo_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES realtime_bayes_dense_reconstruction
  CATKIN_DEPENDS cv_bridge image_geometry image_transport pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs message_runtime pointcloud_integrator flir_ptu_driver gazebo_ros  gazebo_msgs message_filters
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})

###########
## Build ##
###########
set(MODELLOADER_SRC src/ModelLoader)
set(COMMON_SRC src/Common)
set(LIVESCAN_SRC src/LiveScan)
set(COLLISIONWARNING_SRC src/CollisionWarning)

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/thirdparty/Sophus)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}

  ${QT_INCLUDES}
  ${EIGEN3_INCLUDE_DIR}
  ${QGLVIEWER_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus

)

LIST(APPEND LINK_LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    #${Sophus_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

# For Qt creator
file(GLOB_RECURSE HEADER_FILES include/*.h)


add_library(model_loader_lib
    ${MODELLOADER_SRC}/ModelLoader.cpp
    ${COMMON_SRC}/Pts.cpp
    ${COMMON_SRC}/Ply.cpp
    #${COMMON_SRC}/Ptx.cpp
)

add_library(live_scan_lib
    ${LIVESCAN_SRC}/LiveScan.cpp
    ${COMMON_SRC}/Pts.cpp
    ${COMMON_SRC}/Ply.cpp
    ${COMMON_SRC}/ScanPlans2PointCloud.cpp
)

add_library(collision_warning_lib
    ${COLLISIONWARNING_SRC}/CollisionWarning.cpp
    ${COMMON_SRC}/ScanPlans2PointCloud.cpp
)


## Declare a cpp executable
add_executable(model_loader ${MODELLOADER_SRC}/main.cpp ${HEADER_FILES})
target_link_libraries(model_loader_lib ${LINK_LIBS})
target_link_libraries(model_loader model_loader_lib)

add_executable(live_scan ${LIVESCAN_SRC}/main.cpp ${HEADER_FILES})
target_link_libraries(live_scan_lib ${LINK_LIBS})
target_link_libraries(live_scan live_scan_lib)

add_executable(collision_warning ${COLLISIONWARNING_SRC}/main.cpp ${HEADER_FILES})
target_link_libraries(collision_warning_lib ${LINK_LIBS})
target_link_libraries(collision_warning collision_warning_lib)


