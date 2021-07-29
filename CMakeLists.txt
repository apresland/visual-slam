cmake_minimum_required(VERSION 3.0)
project(visual-slam)

set(CMAKE_CXX_FLAGS "-std=c++17")

add_subdirectory(test_eigen)
add_subdirectory(stereo_vision)

