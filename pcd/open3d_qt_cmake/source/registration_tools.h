#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include "open3d/Open3D.h"
using namespace open3d;


std::tuple<std::shared_ptr<geometry::PointCloud>,
    std::shared_ptr<pipelines::registration::Feature>>
    PreprocessPointCloud(std::shared_ptr<geometry::PointCloud> pcd, float voxel_size = 2.0);

pipelines::registration::RegistrationResult fpfh_registration(
    std::shared_ptr<geometry::PointCloud> source,
    std::shared_ptr<geometry::PointCloud> target,
    std::shared_ptr<pipelines::registration::Feature> source_fpfh,
    std::shared_ptr<pipelines::registration::Feature> target_fpfh,
    float voxel_size,
    utility::optional<unsigned int> seed_ = utility::nullopt,
    bool mutual_filter = true);



pipelines::registration::RegistrationResult icp_registration(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    float icp_distance_threshold, const Eigen::Matrix4d& init_matrix = Eigen::Matrix4d::Identity(),int icp_method = 2, int icp_iteration = 30);


