cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_integrator)

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
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/PointCloudIntegratorParams.cfg
)

#Libraries for depthfilter
find_package(OpenGL REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
find_package(QGLViewer REQUIRED)
find_package(Eigen3 REQUIRED)
#FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   meshMsg.msg
   scanMsg.msg
   keyScanMsg.msg
   keyScanGraphMsg.msg

   camdepthMsg.msg
   keyFrameMsg.msg
   keyFrameGraphMsg.msg
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  snapshot.srv
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
  std_msgs
  sensor_msgs
  geometry_msgs
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
  CATKIN_DEPENDS cv_bridge image_geometry image_transport pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs geometry_msgs message_runtime dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_library(pointcloud_integrator_lib
    src/PointCloudViewer.cpp
    src/KeyFrameDisplay.cpp
    src/KeyFrameGraphDisplay.cpp
    src/settings.cpp
    src/ndt_omp.cpp
    src/voxel_grid_covariance_omp.cpp
)

#set(SOURCE_FILES
#  src/PointCloudViewer.cpp
#  src/KeyFrameDisplay.cpp
#  src/KeyFrameGraphDisplay.cpp
#  src/settings.cpp
#)
#
#set(HEADER_FILES
#  src/PointCloudViewer.h
#  src/KeyFrameDisplay.h
#  src/KeyFrameGraphDisplay.h
#  src/settings.h
#)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

  ${QT_INCLUDES}
  ${EIGEN3_INCLUDE_DIR}
  ${QGLVIEWER_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
)

set(Sophus_LIBRARIES libSophus.so)
LIST(APPEND LINK_LIBS
  ${catkin_LIBRARIES}
  #${OpenCV_LIBRARIES}
  #${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${QT_LIBRARIES}
  ${QGLVIEWER_LIBRARY} GL glut GLU
)

##### original #####
#rosbuild_add_executable(viewer src/main_viewer.cpp ${source_files} ${header_files})
#rosbuild_link_boost(viewer thread)
#rosbuild_add_executable(videostitch src/main_stitchvideos.cpp)

## For Qt creator
#file(glob_recurse header_files include/*.h)

## Declare a cpp executable
add_executable(pointcloud_integrator src/main_viewer.cpp ${SOURCE_FILES} ${HEADER_FILES})


add_dependencies(pointcloud_integrator ${PROJECT_NAME}_gencfg)

#add_dependencies(pointcloud_integrator ${PROJECT_NAME}_gencfg ${pointcloud_integrator_EXPORTED_TARGETS})

#target_link_libraries(pointcloud_viewer ${qglviewer_libraries} ${qt_libraries} ${qglviewer_library} gl glut glu)
target_link_libraries(pointcloud_integrator_lib ${LINK_LIBS})
target_link_libraries(pointcloud_integrator pointcloud_integrator_lib)