cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(compare_pcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (com_pcd com_pcd.cpp)
target_link_libraries (com_pcd ${PCL_LIBRARIES})
