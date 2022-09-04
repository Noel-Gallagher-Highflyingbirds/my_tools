cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(Euler2rt)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_executable (euler2rt euler2rt.cpp)
add_executable (rt2euler rt2euler.cpp)

