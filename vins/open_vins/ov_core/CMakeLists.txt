cmake_minimum_required(VERSION 3.3)
project(ov_core)

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(OpenCV 3 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
endif ()

# Find Eigen package
find_package(Eigen3 REQUIRED)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# Output some information about our dependencies
message(STATUS "OPENCV: ${OpenCV_VERSION} | BOOST: ${Boost_VERSION} | EIGEN3: ${EIGEN3_VERSION}")

# Add include directories
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  # Include other directories as needed
)

# If we will compile with aruco support
option(ENABLE_ARUCO_TAGS "Enable or disable aruco tag (disable if no contrib modules)" ON)
if (NOT ENABLE_ARUCO_TAGS)
    add_definitions(-DENABLE_ARUCO_TAGS=0)
    message(WARNING "DISABLING ARUCOTAG TRACKING!")
else ()
    add_definitions(-DENABLE_ARUCO_TAGS=1)
endif ()

# Check if Python libs are found
find_package(PythonLibs QUIET)
option(DISABLE_MATPLOTLIB "Disable or enable matplotlib plot scripts in ov_eval" OFF)
if (PYTHONLIBS_FOUND AND NOT DISABLE_MATPLOTLIB)
    add_definitions(-DHAVE_PYTHONLIBS=1)
    message(STATUS "PYTHON VERSION: ${PYTHONLIBS_VERSION_STRING}")
    message(STATUS "PYTHON INCLUDE: ${PYTHON_INCLUDE_DIRS}")
    message(STATUS "PYTHON LIBRARIES: ${PYTHON_LIBRARIES}")
    include_directories(${PYTHON_INCLUDE_DIRS})
    list(APPEND thirdparty_libraries ${PYTHON_LIBRARIES})
endif ()

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

# Find our ROS version!
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if (catkin_FOUND)
    message(STATUS "ROS 1 version found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND)
    message(STATUS "ROS 2 version found, building ROS2.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else ()
    message(STATUS "No ROS versions found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
endif ()
