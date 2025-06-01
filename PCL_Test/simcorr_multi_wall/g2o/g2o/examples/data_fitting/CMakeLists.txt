cmake_minimum_required(VERSION 3.5)
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
FIND_PATH(CSPARSE_INCLUDE_DIR NAMES cs.h
  PATHS
  /usr/include/suitesparse
  /usr/include
  /opt/local/include
  /usr/local/include
  /sw/include
  /usr/include/ufsparse
  /opt/local/include/ufsparse
  /usr/local/include/ufsparse
  /sw/include/ufsparse
)


link_directories(${G2O_LIBRARY_DIRS})
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)


ADD_EXECUTABLE(circle_fit circle_fit.cpp)
SET_TARGET_PROPERTIES(circle_fit PROPERTIES OUTPUT_NAME circle_fit${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(circle_fit core solver_csparse)

