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

bool voxel_downsample = true;

std::tuple<std::shared_ptr<geometry::PointCloud>,
           std::shared_ptr<pipelines::registration::Feature>>
PreprocessPointCloud(const std::string file_name, float voxel_size = 2.0) {
    //从文件读取点云
    auto pcd = open3d::io::CreatePointCloudFromFile(file_name);
    std::cout << "read " << pcd->points_.size() << " points from " << file_name
        << std::endl;
    if (voxel_downsample) {
        //降采样
        pcd = pcd->VoxelDownSample(voxel_size);
        std::cout << "voxel_size=" << voxel_size << ", after downsample "
            << pcd->points_.size() << "points left" << endl;
    }
    //计算法向量
    clock_t EstimateNormals_time_s = clock();
    pcd->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2, 30));
    clock_t EstimateNormals_time = clock();
    std::cout << "EstimateNormals costs: " << (double)(EstimateNormals_time - EstimateNormals_time_s) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;
    
    /*
    https://github.com/isl-org/Open3D/blob/master/cpp/open3d/geometry/PointCloud.h#L239
    void OrientNormalsToAlignWithDirection(
    const Eigen::Vector3d& orientation_reference =
        Eigen::Vector3d(0.0, 0.0, 1.0));
    https://github.com/isl-org/Open3D/blob/7c62640441e3da18bcbe146723ed83ff544b2fbb/cpp/open3d/geometry/EstimateNormals.cpp#L338
    */
    //指定法向量方向
    pcd->OrientNormalsToAlignWithDirection();
    //计算fpfh特征
    clock_t ComputeFPFHFeature_time_s = clock();
    auto pcd_fpfh = pipelines::registration::ComputeFPFHFeature(
            *pcd,
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));
    clock_t ComputeFPFHFeature_time = clock();
    std::cout << "ComputeFPFHFeature costs: " << (double)(ComputeFPFHFeature_time - ComputeFPFHFeature_time_s) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;

    return std::make_tuple(pcd, pcd_fpfh);
}


void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();

    std::cout << "Input parameters:" << std::endl
        << "--path_source path: path to a pcd need to be register" << std::endl
        << "--path_target path: path to the target pcd" << std::endl
        << "--voxel_size v: voxel size to downsample pcd" << std::endl
        << "--use_icp iteration(optional): use icp to fine tuning, and set iteration(default iteration=30)" << std::endl
        << "--icp_threshold_ratio r(optional): the final icp threshold is ratio*voxel_size(default ratio = 1.5, icp_threshold=1.5*voxel_size)" << std::endl
        << "--icp_method m(optional): point to point(0), point to plane(1), generalized icp(2), default 0" << std::endl
        << "--visualize(optional): visualize results" << std::endl
        << "--fix_seed seed(optional): use fix seed in ransac step for deterministic result,(default seed=123456)" << std::endl
        << "--no_mutual_filter(optional): mutual filter will not be used" << std::endl
        << "--origin_color(optional): show origin color when show the registration result" << std::endl
        << "--no_downsample(optional): voxel downsample will not be used" << std::endl;
}




int main(int argc, char* argv[]) {
    using namespace open3d;
    clock_t total_time_s = clock();

    std::string path_source = "";
    std::string path_target = "";
    //set voxel_size
    float voxel_size;
    float distance_threshold;//fpfh distance_threshold

    //use icp to fine tuning
    bool use_icp = false;
    int icp_iteration = 30;

    float icp_distance_threshold = distance_threshold;
    //icp_method(optional): point to point(0), point to plane(1), generalized icp(2), default 0
    int icp_method = 0;

    bool visualize = false;
    clock_t start_time = clock();

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 4 ||
        utility::ProgramOptionExistsAny(argc, argv, { "-h", "--help" })) {
        PrintHelp();
        return 1;
    }
    //file path
    path_source=utility::GetProgramOptionAsString(argc, argv, "--path_source", "");
    if (path_source == "") {
        std::cout << "please check your source pcd path: "<< path_source << std::endl;
        return 0;
    }
    path_target = utility::GetProgramOptionAsString(argc, argv, "--path_target", "");
    if (path_target == "") {
        std::cout << "please check your target pcd path: "<< path_target << std::endl;
        return 0;
    }

    //voxel size
    voxel_size = utility::GetProgramOptionAsDouble(argc, argv, "--voxel_size", 2.0);
    distance_threshold = voxel_size * 1.5;//fpfh distance_threshold
    //use icp
    if (utility::ProgramOptionExists(argc, argv, "--use_icp")) {
        use_icp = true;
        icp_iteration = utility::GetProgramOptionAsInt(argc, argv, "--use_icp", 30);
    }
    //icp distance threshold
    icp_distance_threshold = voxel_size * utility::GetProgramOptionAsDouble(argc, argv, "--icp_threshold_ratio", 1.5);
    //icp method
    icp_method = utility::GetProgramOptionAsInt(argc, argv, "--icp_method", 0);

    // visualize
    if (utility::ProgramOptionExists(argc, argv, "--visualize")) {
        visualize = true;
    }
    //seed
    utility::optional<unsigned int> seed_ = utility::nullopt;
    if (utility::ProgramOptionExists(argc, argv, "--fix_seed")) {
        seed_ = utility::GetProgramOptionAsInt(argc, argv, "--fix_seed", 123456);
    }

    bool mutual_filter = true;
    if (utility::ProgramOptionExists(argc, argv, "--no_mutual_filter")) {
        mutual_filter = false;
    }

    if (utility::ProgramOptionExists(argc, argv, "--no_downsample")) {
        voxel_downsample = false;
    }
    bool ori_color = false;
    if (utility::ProgramOptionExists(argc, argv, "--ori_color")) {
        ori_color = true;
    }


    // Prepare input
    std::shared_ptr<geometry::PointCloud> source, target;
    std::shared_ptr<pipelines::registration::Feature> source_fpfh, target_fpfh;


    std::shared_ptr<geometry::PointCloud> source_ori(new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target_ori(new geometry::PointCloud);
    open3d::io::ReadPointCloud(path_source, *source_ori);
    open3d::io::ReadPointCloud(path_target, *target_ori);

    std::tie(source, source_fpfh) = PreprocessPointCloud(path_source, voxel_size);
    std::tie(target, target_fpfh) = PreprocessPointCloud(path_target, voxel_size);

    pipelines::registration::RegistrationResult registration_result;
    clock_t pre_proccess_time = clock();
    std::cout << "pre_proccess costs: " << (double)(pre_proccess_time - start_time) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;


    // Prepare checkers
    std::vector<std::reference_wrapper<
        const pipelines::registration::CorrespondenceChecker>>
        correspondence_checker;
    auto correspondence_checker_edge_length =
        pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
            0.9);
    auto correspondence_checker_distance =
        pipelines::registration::CorrespondenceCheckerBasedOnDistance(
            distance_threshold);

    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);


    registration_result = pipelines::registration::
        RegistrationRANSACBasedOnFeatureMatching(
            *source, *target, *source_fpfh, *target_fpfh,
            mutual_filter, distance_threshold,
            pipelines::registration::
            TransformationEstimationPointToPoint(false),
            3, correspondence_checker,
            pipelines::registration::RANSACConvergenceCriteria(1000000, 0.999), seed_);
    //std::cout << "*****debug*****\n\n\n\n";
    std::cout << "*****fpfh_result*****\n";
    cout << endl
        << "fpfh matrix:" << endl
        << registration_result.transformation_ << endl;
    cout << "inlier(correspondence_set size):"
        << registration_result.correspondence_set_.size() << endl;
    std::cout << "\nfitness_(For RANSAC: inlier ratio (# of inlier correspondences / # of all correspondences)): " << registration_result.fitness_ << std::endl
        << "RMSE of all inlier correspondences: " << registration_result.inlier_rmse_ << std::endl;


    clock_t ransac_time = clock();
    std::cout << "ransac costs: " << (double)(ransac_time - pre_proccess_time) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;

    clock_t total_fpfh_time = clock();
    std::cout << "total_fpfh_time: " << (double)(total_fpfh_time - start_time) / (double)CLOCKS_PER_SEC << " s" << std::endl;

    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
        new geometry::PointCloud);
    *source_transformed_ptr = *source;
    source_transformed_ptr->Transform(registration_result.transformation_);
    if (visualize) {
        std::vector<std::pair<int, int>> correspondences_mutual_filter;
        for (int m = 0; m < registration_result.correspondence_set_mutulfilter.size();
            ++m) {
            std::pair<int, int> pair_(0, 0);
            pair_.first = registration_result.correspondence_set_mutulfilter[m][0];
            pair_.second = registration_result.correspondence_set_mutulfilter[m][1];
            correspondences_mutual_filter.push_back(pair_);
        }
        // std::shared_ptr<open3d::geometry::LineSet>
        auto mutual_filter_lineset =
            geometry::LineSet::CreateFromPointCloudCorrespondences(
                *source, *target, correspondences_mutual_filter);


        std::vector<std::pair<int, int>> correspondences_ransac;
        for (int m = 0; m < registration_result.correspondence_set_.size();
            ++m) {
            std::pair<int, int> pair_(0, 0);
            pair_.first = registration_result.correspondence_set_[m][0];
            pair_.second = registration_result.correspondence_set_[m][1];
            correspondences_ransac.push_back(pair_);
        }
        // std::shared_ptr<open3d::geometry::LineSet>
        auto ransac_lineset =
            geometry::LineSet::CreateFromPointCloudCorrespondences(
                *source, *target, correspondences_ransac);
        std::shared_ptr<geometry::PointCloud> source_ptr(
            new geometry::PointCloud);
        std::shared_ptr<geometry::PointCloud> target_ptr(
            new geometry::PointCloud);

        *target_ptr = *target;
        *source_ptr = *source;
        target->PaintUniformColor({ 1, 0, 0 });                  //红
        source->PaintUniformColor({ 0, 1, 0 });                  //绿
        source_transformed_ptr->PaintUniformColor({ 0, 0, 1 });  //蓝



        //visualization::DrawGeometries(
        //    { target, source, source_transformed_ptr, mutual_filter_lineset },
        //    "fpfh correspondences after mutual filter", 960, 900, 960, 100);
        visualization::DrawGeometries(
            { target, source, mutual_filter_lineset },
            "fpfh correspondences after mutual filter", 960, 900, 960, 100);

        //visualization::DrawGeometries(
        //    { target, source, source_transformed_ptr, ransac_lineset },
        //    "fpfh correspondences after ransace", 960, 900, 960, 100);
        visualization::DrawGeometries(
            { target, source, ransac_lineset },
            "fpfh correspondences after ransac", 960, 900, 960, 100);

        if (!ori_color) {
            target_ori->PaintUniformColor({ 1, 0, 0 });                  //红
            source_ori->PaintUniformColor({ 0, 0, 1 });  //蓝
        }

        std::shared_ptr<geometry::PointCloud> source_ori_after_fpfh(new geometry::PointCloud);
        *source_ori_after_fpfh = *source_ori;
        source_ori_after_fpfh->Transform(registration_result.transformation_);
        visualization::DrawGeometries(
            { target_ori, source_ori_after_fpfh },
            "fpfh ransac registration origin pcd", 960, 900, 960, 100);
    }

    if (use_icp) {
        std::shared_ptr<geometry::PointCloud> source_after_fpfh(new geometry::PointCloud);
        // Prepare checkers
        /*
        * ICPConvergenceCriteria(double relative_fitness = 1e-6,
        * double relative_rmse = 1e-6,
        * int max_iteration = 30)
        */

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
        auto _criteria_icp =
            pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, icp_iteration);

        clock_t icp_time_s = clock();
        pipelines::registration::RegistrationResult registration_result_icp;

        if (icp_method == 0) {
            registration_result_icp = open3d::pipelines::registration::RegistrationICP(
                *source_transformed_ptr, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
                pipelines::registration::TransformationEstimationPointToPoint(), _criteria_icp);
        }
        else if (icp_method == 1) {
            registration_result_icp = open3d::pipelines::registration::RegistrationICP(
                *source_transformed_ptr, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
                pipelines::registration::TransformationEstimationPointToPlane(), _criteria_icp);
        
        }
        else if (icp_method == 2) {
            registration_result_icp = open3d::pipelines::registration::RegistrationGeneralizedICP(
                *source_transformed_ptr, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
                pipelines::registration::TransformationEstimationForGeneralizedICP(), _criteria_icp);

        }
        clock_t icp_time = clock();


        std::cout << "icp cost: " << (double)(icp_time - icp_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
        source_transformed_ptr->PaintUniformColor({ 0, 1, 0 });  //蓝

        std::shared_ptr<geometry::PointCloud> pcd_after_fpfh_icp(
            new geometry::PointCloud);
        *pcd_after_fpfh_icp = *source_transformed_ptr;
        pcd_after_fpfh_icp->Transform(registration_result_icp.transformation_);
        pcd_after_fpfh_icp->PaintUniformColor({ 0,0,1 });
        std::vector<std::pair<int, int>> correspondences_icp;
        for (int m = 0; m < registration_result_icp.correspondence_set_.size();
            ++m) {
            std::pair<int, int> pair_(0, 0);
            pair_.first = registration_result_icp.correspondence_set_[m][0];
            pair_.second = registration_result_icp.correspondence_set_[m][1];
            correspondences_icp.push_back(pair_);
        }
        std::cout << "*****fpfh_icp_result*****\n";
        std::cout << std::endl
            << "icp based on fpfh matrix:" << std::endl
            << registration_result_icp.transformation_ << std::endl;
        std::cout << "inlier(correspondence_set size):"
            << registration_result_icp.correspondence_set_.size() <<
            " all(pcd size after voxel downsample): " << source_transformed_ptr->points_.size() << std::endl;

        std::cout << "\nfitness_(# of inlier correspondences / # of points in target): " << registration_result_icp.fitness_ << std::endl
            << "RMSE of all inlier correspondences: " << registration_result_icp.inlier_rmse_ << std::endl;


        Eigen::Matrix4d trans_fpfh_icp = registration_result_icp.transformation_ * registration_result.transformation_;
        std::cout << std::endl
            << "fpfh + icp registration matrix:" << std::endl
            << trans_fpfh_icp << std::endl;



        //// std::shared_ptr<open3d::geometry::LineSet>
        auto icp_lineset =
            geometry::LineSet::CreateFromPointCloudCorrespondences(
                *source_transformed_ptr, *target, correspondences_icp);
        if (visualize) {
            visualization::DrawGeometries(
                { target, source_transformed_ptr, pcd_after_fpfh_icp, icp_lineset },
                "icp result pairs", 960, 900, 960, 100);

            //target_ori->PaintUniformColor({ 1, 0, 0 });                  //红
            //source_ori->PaintUniformColor({ 0, 0, 1 });  //蓝
            std::shared_ptr<geometry::PointCloud> source_ori_after_fpfh_icp(new geometry::PointCloud);
            *source_ori_after_fpfh_icp = *source_ori;
            source_ori_after_fpfh_icp->Transform(trans_fpfh_icp);
            visualization::DrawGeometries(
                { target_ori, source_ori_after_fpfh_icp },
                "icp result ori pcd", 960, 900, 960, 100);
        }

    }
    clock_t total_time = clock();
    std::cout << "total costs: " << (double)(total_time - total_time_s) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;

    return 0;
}
