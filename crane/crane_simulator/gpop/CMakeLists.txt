cmake_minimum_required(VERSION 3.5)

add_definitions("-std=c++11")

set(SRC_DIR src)
set(INCLUDE_DIR ./include/Gpop)
include_directories(${INCLUDE_DIR})

add_library(Gpop ${SRC_DIR}/Bar.cpp
	               ${SRC_DIR}/Gnuplot.cpp
				   ${SRC_DIR}/Pie.cpp
				   ${SRC_DIR}/Series.cpp
				   ${SRC_DIR}/Vector.cpp
				   ${SRC_DIR}/Hist.cpp
				   )

install(TARGETS Gpop
	    RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		)

install(
		DIRECTORY include/Gpop
		DESTINATION /usr/local/include
		)
