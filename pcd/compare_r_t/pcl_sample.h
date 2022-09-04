#pragma once
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void down_sample_voxel(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_filtered, float grid_size, bool origin_point = 1);

void down_sample_random(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_filtered, int random_subsample = 2000);

void down_sample_uniform(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_filtered, float grid_size = 2.0);

