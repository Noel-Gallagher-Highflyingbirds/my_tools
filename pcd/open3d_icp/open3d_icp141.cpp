// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "open3d/Open3D.h"

using namespace open3d;
using namespace std;





void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();

    std::cout << "Input parameters:" << std::endl
        << "source_pcd: path to a pcd need to be register" << std::endl
        << "target_pcd: path to the target pcd" << std::endl
        << "voxel_size: voxel size to downsample pcd" << std::endl
        << "--visualize: visualize pcd" << std::endl
        << "--threshold: visualize pcd" << std::endl
        << "--no_downsample: voxel downsample will not be used" << std::endl;

}

int main(int argc, char* argv[]) {
    using namespace open3d;

    clock_t start_time = clock();

    std::string path_source;
    std::string path_target;
    double voxel_size = 0;
    double threshold = 0;
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 5 || utility::ProgramOptionExistsAny(argc, argv, { "-h", "--help" })) {
        PrintHelp();
        return 1;
    }
    path_source = utility::GetProgramOptionAsString(argc, argv, "--path_source", "");
    if (path_source == "") {
        std::cout << "please check your source pcd path: " << path_source << std::endl;
        return 0;
    }
    path_target = utility::GetProgramOptionAsString(argc, argv, "--path_target", "");
    if (path_target == "") {
        std::cout << "please check your target pcd path: " << path_target << std::endl;
        return 0;
    }
    //voxel size
    voxel_size = utility::GetProgramOptionAsDouble(argc, argv, "--voxel_size", 2.0);
    threshold = utility::GetProgramOptionAsDouble(argc, argv, "--threshold",5.0);


    bool visualize = false;
    if (utility::ProgramOptionExists(argc, argv, "--visualize")) {
        visualize = true;
    }
    //visualize = true;
    utility::optional<unsigned int> seed_ = utility::nullopt;
    if (utility::ProgramOptionExists(argc, argv, "--fix_seed")) {
        seed_ = 123456;
    }

    std::cout << voxel_size << std::endl;


    

    // Prepare input
    std::shared_ptr<geometry::PointCloud> source, target, source_down, target_down;

    
    source = open3d::io::CreatePointCloudFromFile(path_source);
    target = open3d::io::CreatePointCloudFromFile(path_target);

    std::cout << "read " << source->points_.size() << " points from " << path_source << std::endl;
    std::cout << "read " << target->points_.size() << " points from " << path_target << std::endl;


    //降采样
    clock_t downsample_time_s = clock();
    source_down = source->VoxelDownSample(voxel_size);
    std::cout << "voxel_size=" << voxel_size << ", after downsample source, "
        << source->points_.size() << "points left" << endl;
    target_down = target->VoxelDownSample(voxel_size);
    std::cout << "voxel_size=" << voxel_size << ", after downsample target, "
        << target->points_.size() << "points left" << endl;

    clock_t downsample_time = clock();
    std::cout << "downsample costs: " << (double)(downsample_time - downsample_time_s) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;



    // Prepare checkers
    /*
    * ICPConvergenceCriteria(double relative_fitness = 1e-6,
    * double relative_rmse = 1e-6,
    * int max_iteration = 30)
    */
    auto _criteria_icp =
        pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 30);
    /*
    * /// \brief Functions for ICP registration.
    ///
    /// \param source The source point cloud.
    /// \param target The target point cloud.
    /// \param max_correspondence_distance Maximum correspondence points-pair
    /// distance. \param init Initial transformation estimation.
    ///  Default value: array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.],
    ///  [0., 0., 0., 1.]])
    /// \param estimation Estimation method.
    /// \param criteria Convergence criteria.
    RegistrationResult RegistrationICP(
            const geometry::PointCloud &source,
            const geometry::PointCloud &target,
            double max_correspondence_distance,
            const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(),
            const TransformationEstimation &estimation =
                    TransformationEstimationPointToPoint(false),
            const ICPConvergenceCriteria &criteria = ICPConvergenceCriteria());
    */
	clock_t icp_time_s = clock();
    pipelines::registration::RegistrationResult
        registration_result_icp = open3d::pipelines::registration::RegistrationICP(
            *source_down, *target_down, threshold, Eigen::Matrix4d::Identity(),
            pipelines::registration::TransformationEstimationPointToPoint(), _criteria_icp);
	clock_t icp_time = clock();
	std::cout << "icp cost: " << (double)(icp_time - icp_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;


    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
        new geometry::PointCloud);
    *source_transformed_ptr = *source_down;

    source_transformed_ptr->Transform(registration_result_icp.transformation_);

    std::vector<std::pair<int, int>> correspondences_icp;
    for (int m = 0; m < registration_result_icp.correspondence_set_.size();
        ++m) {
        std::pair<int, int> pair_(0, 0);
        pair_.first = registration_result_icp.correspondence_set_[m][0];
        pair_.second = registration_result_icp.correspondence_set_[m][1];
        correspondences_icp.push_back(pair_);
    }
    std::cout << "*****icp_result*****\n";
    std::cout << std::endl
        << "icp matrix:" << std::endl
        << registration_result_icp.transformation_ << std::endl;
    std::cout << "inlier(correspondence_set size):"
        << registration_result_icp.correspondence_set_.size() <<
        " all(pcd size after voxel downsample): " << source_down->points_.size() << std::endl;

    std::cout << "\nfitness_(# of inlier correspondences / # of points in target): " << registration_result_icp.fitness_ << std::endl
        << "RMSE of all inlier correspondences: " << registration_result_icp.inlier_rmse_ << std::endl;


    //// std::shared_ptr<open3d::geometry::LineSet>
    auto icp_lineset =
        geometry::LineSet::CreateFromPointCloudCorrespondences(
            *source_down, *target_down, correspondences_icp);

    target_down->PaintUniformColor({ 1,0,0 });
    source_down->PaintUniformColor({ 0,1,0 });
    source_transformed_ptr->PaintUniformColor({ 0,0,1 });

    if (visualize) {
        visualization::DrawGeometries(
            { source_down, target_down, source_transformed_ptr, icp_lineset },
            "icp result pairs", 960, 900, 960, 100);

        target->PaintUniformColor({ 1,0,0 });
        source->PaintUniformColor({ 0,1,0 });
        source->Transform(registration_result_icp.transformation_);

        visualization::DrawGeometries(
            { source, target},
            "icp result", 960, 900, 960, 100);
    }

    

    return 0;
}
