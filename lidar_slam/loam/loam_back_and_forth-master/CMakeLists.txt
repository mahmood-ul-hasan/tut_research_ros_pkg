cmake_minimum_required(VERSION 2.8.3)
project(loam_back_and_forth)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)


include_directories(
  #include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  #INCLUDE_DIRS include
)

add_executable(scanRegistration_bf src/scanRegistration.cpp)
target_link_libraries(scanRegistration_bf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserOdometry_bf src/laserOdometry.cpp)
target_link_libraries(laserOdometry_bf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserMapping_bf src/laserMapping.cpp)
target_link_libraries(laserMapping_bf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(transformMaintenance_bf src/transformMaintenance.cpp)
target_link_libraries(transformMaintenance_bf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


#if (CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  # TODO: Download test data
#  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
#    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
#    MD5 3d5194e6981975588b7a93caebf79ba4)
#  add_custom_target(${PROJECT_NAME}_test_data
#    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
#    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
#  configure_file(tests/loam.test.in
#                 ${PROJECT_BINARY_DIR}/test/loam.test)
#  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
#    DEPENDENCIES
#      ${PROJECT_NAME}_test_data
#      scanRegistration
#      laserOdometry
#      laserMapping
#      transformMaintenance)
#endif()


