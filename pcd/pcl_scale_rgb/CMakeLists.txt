cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(PCL_SCALE_RGB)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})		
add_executable (pcl_scale_rgb pcl_scale_rgb.cpp)
target_link_libraries (pcl_scale_rgb ${PCL_LIBRARIES})

