cmake_minimum_required(VERSION 3.20)

project(controllers CXX)

add_library(pid pid.cpp pid.h)
target_include_directories(pid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package(jsoncpp CONFIG REQUIRED)
add_library(jsoninterface jsoninterface.h jsoninterface.cpp)
target_include_directories(jsoninterface PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(jsoninterface PUBLIC jsoncpp_lib jsoncpp_object JsonCpp::JsonCpp)

add_executable(pid_exe pid_exe.cpp)
target_link_libraries(pid_exe PUBLIC pid jsoninterface)
