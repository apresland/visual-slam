set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Debug")

# catkin interop
set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}/devel")
set(CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/devel;/opt/ros/melodic/;/opt/ros/melodic/lib;/opt/ros/melodic/share;")


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

# ROS
find_package(catkin REQUIRED
        COMPONENTS roscpp std_msgs image_transport cv_bridge
        )
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
        CATKIN_DEPENDS std_msgs image_transport cv_bridge
)

add_executable(${PROJECT_NAME}
        system.cpp
        system.h
        context.h
        frontend.cpp
        frontend.h
        backend.cpp
        backend.h
        sequence.cpp
        sequence.h
        vizualization.cpp
        vizualization.h)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${catkin_LIBRARIES}
        ceres
        g2o_core
        g2o_stuff
        g2o_types_sba
        g2o_types_slam3d
        pthread
        fmt)

include_directories(".")

add_subdirectory(map)
add_subdirectory(optimize)
add_subdirectory(sensor)
add_subdirectory(solve)
