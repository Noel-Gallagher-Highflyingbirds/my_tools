cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(PCL_CONVERT_INTENSITY)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})		
add_executable (pcl_convert_intensity pcl_convert_intensity.cpp)
target_link_libraries (pcl_convert_intensity ${PCL_LIBRARIES})

