cmake_minimum_required(VERSION 3.5)

add_definitions("-std=c++14 -fPIC")

include_directories(./include)

#add_subdirectory(test)

set(SRC_DIR src)
set(INCLUDE_DIR include/fanda)

add_library(fanda 
	${SRC_DIR}/File.cpp
	${SRC_DIR}/String.cpp
	${SRC_DIR}/Zip.cpp
	${SRC_DIR}/Csv.cpp
	)

install(TARGETS fanda
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)

install(DIRECTORY include/fanda
	DESTINATION /usr/local/include
	)
