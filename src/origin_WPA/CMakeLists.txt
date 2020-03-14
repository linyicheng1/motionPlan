cmake_minimum_required(VERSION 3.15)
project(WPA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

include_directories(.)

add_executable(WPA
        WolfPackAlgorithm.cpp
        WPA.cpp
        WPA.h)
