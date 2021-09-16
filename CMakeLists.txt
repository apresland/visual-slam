set(CMAKE_CXX_STANDARD 14)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++14")
project(visual-slam)

set(CMAKE_BUILD_TYPE "Debug")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
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

# ceres
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

# g2o
find_package(g2o REQUIRED)
include_directories(${g2o_INCLUDE_DIRS})

# CSparse
#find_package( CSparse REQUIRED )
#include_directories( ${CSPARSE_INCLUDE_DIR} )

include_directories("include")

add_executable(stereo
        src/main.cpp
        src/system.cpp
        src/frontend.cpp
        src/backend.cpp
        src/map.cpp
        src/sequence.cpp
        src/camera.cpp
        src/detector.cpp
        src/matcher.cpp
        src/tracker.cpp
        src/triangulator.cpp
        src/estimation.cpp
        src/optimization.cpp
        src/viewer.cpp)
target_link_libraries(stereo
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        g2o_core
        g2o_stuff
        g2o_types_sba
        g2o_types_slam3d
        pthread fmt)