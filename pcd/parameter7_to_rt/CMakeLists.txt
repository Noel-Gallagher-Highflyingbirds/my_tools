cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(parameter7_to_rt)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})		
add_executable (parameter7_to_rt parameter7_to_rt.cpp)
target_link_libraries (parameter7_to_rt ${PCL_LIBRARIES})

