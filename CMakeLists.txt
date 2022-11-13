cmake_minimum_required(VERSION 3.23)
project(CassyReadout)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(hidapi)
add_executable(CassyReadout
        Cassy.cpp
        Cassy.h main.cpp SerialPort.h SerialPort.cpp)
if(UNIX)
target_link_libraries(CassyReadout PRIVATE hidapi::hidapi explain)
endif(UNIX)

if(WIN32)
    target_link_libraries(CassyReadout PRIVATE hidapi::hidapi)
endif(WIN32)
