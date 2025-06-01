cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(initial_thres)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (init_thres init_thres.cpp)
target_link_libraries (init_thres ${PCL_LIBRARIES})
