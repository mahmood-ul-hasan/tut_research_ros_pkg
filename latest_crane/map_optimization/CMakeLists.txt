
cmake_minimum_required(VERSION 3.5)
project(map_optimization)

# Can we use C++17 in indigo?
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# pcl 1.7 causes a segfault when it is built with debug mode
# set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  fast_gicp
  geodesy
  geometry_msgs
  interactive_markers
  message_generation
  ndt_omp
  nmea_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf_conversions
)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
# link_libraries(${G2O_LIBRARIES})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_library(VGICP_CUDA_FOUND NAMES fast_vgicp_cuda)
message(STATUS "VGICP_CUDA_FOUND:" ${VGICP_CUDA_FOUND})
if(VGICP_CUDA_FOUND)
  add_definitions(-DUSE_VGICP_CUDA)
endif()





find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})








# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hdl_graph_slam_nodelet
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nmea_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



#file(GLOB SOURCES "src/*.cpp")
add_library(pre_process src/pre_processing.cpp src/key_index.cpp)
#target_link_libraries(pre_process)

add_library(hdl_graph_slam
  src/hdl_graph_slam/graph_slam.cpp
  src/hdl_graph_slam/information_matrix_calculator.cpp
  src/g2o/robust_kernel_io.cpp
  src/graph_optimization.cpp
)

#add_library(graph_optimization src/graph_optimization.cpp)
add_library(ploting_cloud_visualization_troubleshooting src/ploting_cloud_visualization_troubleshooting.cpp)



add_executable (data_corr src/data_corr.cpp )
target_link_libraries (data_corr pre_process hdl_graph_slam ploting_cloud_visualization_troubleshooting
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
  			${G2O_STUFF_LIBRARY}
		  	${G2O_SOLVER_PCG}
  			${G2O_SOLVER_CSPARSE}
  			${G2O_SOLVER_CHOLMOD}
  			${G2O_TYPES_SLAM3D}
  			${G2O_TYPES_SLAM3D_ADDONS}
        ${PYTHON_LIBRARIES})











