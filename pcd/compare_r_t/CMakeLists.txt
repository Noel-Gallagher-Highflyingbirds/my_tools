cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(compare_r_t)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})		
add_executable (compare_r_t compare_r_t.cpp)
target_link_libraries (compare_r_t ${PCL_LIBRARIES})

