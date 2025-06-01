cmake_minimum_required(VERSION 2.8.3)
project(k_crane_control)
set(CMAKE_EXPORT_COMPILE_COMMAND ON)

add_definitions("-std=c++11 -fPIC")

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  trajectory_msgs
  tf2
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ds4toangle src/ds4toangle.cpp)
add_executable(go_initial_pose src/go_initial_pose.cpp)
add_executable(move_trapezoid src/move_trapezoid.cpp)
add_executable(joint_trajectory_action src/joint_trajectory_action.cpp)
add_executable(sano_joint_trajectory_action src/sano_joint_trajectory_action.cpp)
add_executable(integral src/integral.cpp)
add_executable(logger src/logger.cpp)
add_executable(initial_state src/initial_state.cpp)
add_executable(initial_joint4 src/initial_joint4.cpp)
add_executable(initial_joint12 src/initial_joint12.cpp)
add_executable(initial_joint23 src/initial_joint23.cpp)
add_executable(initial_joint_boom src/initial_joint_boom.cpp)
add_executable(initial_joint_boom_pi src/initial_joint_boom_pi.cpp)
add_executable(pause_gazebo src/pause_gazebo.cpp)
add_executable(unpause_gazebo src/unpause_gazebo.cpp)
add_executable(crane_controller_node src/crane_controller.cpp)

target_link_libraries(ds4toangle ${catkin_LIBRARIES})
target_link_libraries(go_initial_pose ${catkin_LIBRARIES})
target_link_libraries(move_trapezoid ${catkin_LIBRARIES})
target_link_libraries(joint_trajectory_action fanda ${catkin_LIBRARIES})
target_link_libraries(sano_joint_trajectory_action fanda ${catkin_LIBRARIES})
target_link_libraries(integral fanda Gpop ${catkin_LIBRARIES})
target_link_libraries(logger ${catkin_LIBRARIES})
target_link_libraries(initial_state ${catkin_LIBRARIES})
target_link_libraries(initial_joint4 ${catkin_LIBRARIES})
target_link_libraries(initial_joint12 ${catkin_LIBRARIES})
target_link_libraries(initial_joint23 ${catkin_LIBRARIES})
target_link_libraries(initial_joint_boom ${catkin_LIBRARIES})
target_link_libraries(initial_joint_boom_pi ${catkin_LIBRARIES})
target_link_libraries(pause_gazebo ${catkin_LIBRARIES})
target_link_libraries(unpause_gazebo ${catkin_LIBRARIES})
target_link_libraries(crane_controller_node ${catkin_LIBRARIES})
