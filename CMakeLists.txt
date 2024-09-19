cmake_minimum_required(VERSION 3.10)
project(opencv_demo)

find_package(OpenCV REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(control src/control.cpp)
add_executable(opencv_demo main.cpp )
target_link_libraries(opencv_demo ${OpenCV_LIBS} control)
