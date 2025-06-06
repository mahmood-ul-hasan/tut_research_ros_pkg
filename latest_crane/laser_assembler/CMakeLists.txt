cmake_minimum_required(VERSION 3.0.2)
project(laser_assembler)

##############################################################################
# Dependencies
##############################################################################

# Define ROS and package dependencies
set(THIS_PACKAGE_ROS_DEPS
  tf
  sensor_msgs
  message_filters
  roscpp
  laser_geometry
  filters
  pcl_ros
  pcl_conversions
)

find_package(catkin REQUIRED COMPONENTS
  ${THIS_PACKAGE_ROS_DEPS}
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED QUIET)

# Optional: Python dependencies (remove if unused)
find_package(PythonLibs REQUIRED)
find_package(rosbag REQUIRED)

##############################################################################
# Include Directories
##############################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS} # Optional: only if Python is required
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# Service Definitions
##############################################################################

add_service_files(
  FILES
  AssembleScans.srv
  AssembleScans2.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

##############################################################################
# Define Package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS} message_runtime
  DEPENDS Boost PCL
)


##############################################################################
# Build
##############################################################################

add_executable(laser_scan_assembler_srv src/laser_scan_assembler_srv.cpp)
target_link_libraries(laser_scan_assembler_srv ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(laser_scan_assembler_srv ${PROJECT_NAME}_gencpp)

add_executable(laser_scan_assembler src/laser_scan_assembler.cpp)
target_link_libraries(laser_scan_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(laser_scan_assembler ${PROJECT_NAME}_gencpp)

add_executable(laser_scan_assembler_after_loop_closure src/laser_scan_assembler_after_loop_closure.cpp)
target_link_libraries(laser_scan_assembler_after_loop_closure ${catkin_LIBRARIES} ${Boost_LIBRARIES}   pcl_common  ${PYTHON_LIBRARIES} ${rosbag_LIBRARIES})
# Add this line

add_dependencies(laser_scan_assembler_after_loop_closure ${PROJECT_NAME}_gencpp)


add_executable(merge_clouds src/merge_clouds.cpp)
target_link_libraries(merge_clouds ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(merge_clouds ${PROJECT_NAME}_gencpp)

add_executable(point_cloud_assembler_srv src/point_cloud_assembler_srv.cpp)
target_link_libraries(point_cloud_assembler_srv ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(point_cloud_assembler_srv ${PROJECT_NAME}_gencpp)

add_executable(point_cloud_assembler src/point_cloud_assembler.cpp)
target_link_libraries(point_cloud_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(point_cloud_assembler ${PROJECT_NAME}_gencpp)

add_executable(point_cloud2_assembler src/point_cloud2_assembler.cpp)
target_link_libraries(point_cloud2_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(point_cloud2_assembler ${PROJECT_NAME}_gencpp)

add_executable(LaserAssemblerVelodyne script/LaserAssemblerVelodyne.cpp)
target_link_libraries(LaserAssemblerVelodyne ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(LaserAssemblerVelodyne ${PROJECT_NAME}_gencpp)



## unit testing

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)

  add_executable(periodic_snapshotter examples/periodic_snapshotter.cpp)
  target_link_libraries(periodic_snapshotter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  add_dependencies(periodic_snapshotter ${PROJECT_NAME}_gencpp)

  add_executable(dummy_scan_producer test/dummy_scan_producer.cpp)
  target_link_libraries(dummy_scan_producer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  add_dependencies(dummy_scan_producer ${PROJECT_NAME}_gencpp)

  add_executable(test_assembler test/test_assembler.cpp)
  target_link_libraries(test_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GTEST_LIBRARIES})
  add_dependencies(test_assembler ${PROJECT_NAME}_gencpp)

  add_rostest(test/test_laser_assembler.launch)
endif()

##############################################################################
# Install
##############################################################################

install(TARGETS
    laser_scan_assembler_srv
    laser_scan_assembler
    laser_scan_assembler_after_loop_closure
    merge_clouds
    point_cloud_assembler_srv
    point_cloud_assembler
    point_cloud2_assembler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
