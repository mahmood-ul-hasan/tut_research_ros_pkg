cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(corr_multi_wall)

find_package(PCL 1.2 REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

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


add_executable (data_corr src/data_corr.cpp)
target_link_libraries (data_corr pre_process hdl_graph_slam
                       	${PCL_LIBRARIES}
                       	${G2O_TYPES_DATA}
                       	${G2O_CORE_LIBRARY}
  			${G2O_STUFF_LIBRARY}
			${G2O_SOLVER_PCG}
  			${G2O_SOLVER_CSPARSE}
  			${G2O_SOLVER_CHOLMOD}
  			${G2O_TYPES_SLAM3D}
  			${G2O_TYPES_SLAM3D_ADDONS})
