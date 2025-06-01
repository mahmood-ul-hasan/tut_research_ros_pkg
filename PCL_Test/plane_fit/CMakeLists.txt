cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(plane_fit)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (plane_fit plane_fit.cpp)
target_link_libraries (plane_fit ${PCL_LIBRARIES})
