cmake_minimum_required(VERSION 3.5)
project(open3d_slam_yaml_io)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)


set(SRC_FILES
  src/parameter_loaders.cpp
)

set(CATKIN_PACKAGE_DEPENDENCIES
  open3d_slam
)
  
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(yaml-cpp REQUIRED)

if (yaml-cpp_FOUND)
    message(STATUS "yaml-cpp found")
    message(STATUS "Include dirs: ${YAML_CPP_INCLUDE_DIR}")
    message(STATUS "Library: ${YAML_CPP_LIBRARIES}")
else()
    message(FATAL_ERROR "yaml-cpp not found  mah")
endif()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    yaml-cpp
    ${PROJECT_NAME} 
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

