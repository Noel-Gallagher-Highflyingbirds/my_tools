cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_fpfh_visualize)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})		
add_executable (pcl_fpfh_visualize pcl_fpfh_visualize.cpp)
target_link_libraries (pcl_fpfh_visualize ${PCL_LIBRARIES})

