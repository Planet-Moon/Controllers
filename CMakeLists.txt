cmake_minimum_required(VERSION 3.20)

project(controllers CXX)

add_library(pid pid.cpp pid.h)
target_include_directories(pid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(pid_exe pid_exe.cpp)
target_link_libraries(pid_exe PUBLIC pid)
