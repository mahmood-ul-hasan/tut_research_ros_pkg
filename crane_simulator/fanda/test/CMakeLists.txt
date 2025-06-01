cmake_minimum_required(VERSION 3.10)

set(DCMAKE_CXX_FLAGS "-g")

add_executable(test_file test_file.cpp ../src/File.cpp ../src/String.cpp)
add_executable(test_zip test_zip.cpp ../src/Zip.cpp)
add_executable(test_string test_string.cpp ../src/String.cpp)
add_executable(test_csv test_csv.cpp ../src/Csv.cpp ../src/String.cpp)
add_executable(test_tcp_client test_tcp_client.cpp)
add_executable(test_tcp_server test_tcp_server.cpp)
add_executable(test_udp_client test_udp_client.cpp)
add_executable(test_udp_server test_udp_server.cpp)

