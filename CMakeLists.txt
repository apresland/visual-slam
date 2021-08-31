cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++17")
project(visual-slam)

set(CMAKE_BUILD_TYPE "Debug")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++17 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(fmt REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories("/usr/local/include/eigen3/")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# S2o
find_package(g2o REQUIRED)
include_directories(${g2o_INCLUDE_DIRS})

include_directories("include")

add_executable(stereo src/slam.cpp src/system.cpp src/frontend.cpp src/sequence.cpp src/camera.cpp src/detector.cpp src/tracker.cpp src/viewer.cpp)
target_link_libraries(stereo
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        pthread fmt)