cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_driver)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

find_package(catkin REQUIRED COMPONENTS
  actionlib
  diagnostic_updater
  roscpp
  roslaunch
  roslint
  rospy
  serial
  sensor_msgs
  tf
  message_generation
)

find_package(Boost REQUIRED)

add_service_files(
  FILES
  sendCommand.srv
  initCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   LIBRARIES flir_ptu_driver
   CATKIN_DEPENDS sensor_msgs serial message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(flir_ptu_driver src/driver.cpp)
target_link_libraries(flir_ptu_driver ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(flir_ptu_node src/node.cpp)
target_link_libraries(flir_ptu_node ${catkin_LIBRARIES} flir_ptu_driver)
add_dependencies(flir_ptu_node flir_ptu_msgs_generate_messages_cpp)
set_target_properties(flir_ptu_node
                      PROPERTIES OUTPUT_NAME ptu_node PREFIX "")

install(TARGETS flir_ptu_driver flir_ptu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/cmd_angles
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Tests
roslint_cpp()
roslint_add_test()
roslaunch_add_file_check(launch/ptu.launch
  DEPENDENCIES flir_ptu_example_urdf flir_ptu_node)
