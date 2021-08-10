cmake_minimum_required(VERSION 2.8)
project(visual-odometry)

set(CMAKE_BUILD_TYPE "Debug")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable(stereo kittislam.cpp odometry.h odometry.cpp frontend.h frontend.cpp frame.h sequence.h sequence.cpp viewer.h viewer.cpp)
target_link_libraries(stereo ${OpenCV_LIBS} pthread)