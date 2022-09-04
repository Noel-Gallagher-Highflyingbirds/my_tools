#include <string>
#include "registration_tools.h"



std::tuple<std::shared_ptr<geometry::PointCloud>,
    std::shared_ptr<pipelines::registration::Feature>>
    PreprocessPointCloud(std::shared_ptr<geometry::PointCloud> pcd, float voxel_size) {
    std::shared_ptr<geometry::PointCloud> pcd_preprocess;

    //������
    pcd_preprocess = pcd->VoxelDownSample(voxel_size);
    std::cout << "voxel_size=" << voxel_size << ", after downsample "
        << pcd_preprocess->points_.size() << "points left" << std::endl;

    //���㷨����
    clock_t EstimateNormals_time_s = clock();
    pcd_preprocess->EstimateNormals(
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
    //ָ������������
    pcd_preprocess->OrientNormalsToAlignWithDirection();
    //����fpfh����
    clock_t ComputeFPFHFeature_time_s = clock();
    auto pcd_fpfh = pipelines::registration::ComputeFPFHFeature(
        *pcd_preprocess,
        open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));
    clock_t ComputeFPFHFeature_time = clock();
    // std::cout << "ComputeFPFHFeature costs: " << (double)(ComputeFPFHFeature_time - ComputeFPFHFeature_time_s) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;

    return std::make_tuple(pcd_preprocess, pcd_fpfh);
}


pipelines::registration::RegistrationResult fpfh_registration(
    std::shared_ptr<geometry::PointCloud> source,
    std::shared_ptr<geometry::PointCloud> target,
    std::shared_ptr<pipelines::registration::Feature> source_fpfh,
    std::shared_ptr<pipelines::registration::Feature> target_fpfh,
    float voxel_size,
    utility::optional<unsigned int> seed_,
    bool mutual_filter) {

    float distance_threshold = 1.5 * voxel_size;

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

    open3d::pipelines::registration::RegistrationResult registration_result;
    registration_result = pipelines::registration::
        RegistrationRANSACBasedOnFeatureMatching(
            *source, *target, *source_fpfh, *target_fpfh,
            mutual_filter, distance_threshold,
            pipelines::registration::
            TransformationEstimationPointToPoint(false),
            3, correspondence_checker,
            pipelines::registration::RANSACConvergenceCriteria(1000000, 0.999), seed_);
    return registration_result;
}


pipelines::registration::RegistrationResult icp_registration(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    float icp_distance_threshold, const Eigen::Matrix4d& init_matrix,int icp_method, int icp_iteration) {
    std::shared_ptr<geometry::PointCloud> source_transformed(new geometry::PointCloud);


    *source_transformed = *source;
    source_transformed->Transform(init_matrix);

    auto _criteria_icp =
        pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, icp_iteration);

    pipelines::registration::RegistrationResult registration_result;

    if (icp_method == 0) {
        registration_result = open3d::pipelines::registration::RegistrationICP(
            *source_transformed, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
            pipelines::registration::TransformationEstimationPointToPoint(), _criteria_icp);
    }
    else if (icp_method == 1) {
        registration_result = open3d::pipelines::registration::RegistrationICP(
            *source_transformed, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
            pipelines::registration::TransformationEstimationPointToPlane(), _criteria_icp);

    }
    else if (icp_method == 2) {
        registration_result = open3d::pipelines::registration::RegistrationGeneralizedICP(
            *source, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
            pipelines::registration::TransformationEstimationForGeneralizedICP(), _criteria_icp);
    }
    std::cout << "icp registration_result.transformation_: \n"<<registration_result.transformation_ << std::endl;

    return registration_result;
}