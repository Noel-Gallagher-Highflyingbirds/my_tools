#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_registration_qt.h"
#include "open3d/Open3D.h"

#include "open3d/visualization/utility/DrawGeometry.h"
#include "open3d/visualization/gui/Application.h"
#include "open3d/visualization/visualizer/GuiVisualizer.h"
#include "open3d/visualization/visualizer/ViewControlWithCustomAnimation.h"
#include "open3d/visualization/visualizer/ViewControlWithEditing.h"
#include "open3d/visualization/visualizer/Visualizer.h"
#include "open3d/visualization/visualizer/VisualizerWithCustomAnimation.h"
#include "open3d/visualization/visualizer/VisualizerWithEditing.h"
#include "open3d/visualization/visualizer/VisualizerWithKeyCallback.h"
#include "open3d/visualization/visualizer/VisualizerWithVertexSelection.h"


using namespace open3d;

class open3d_qt : public QMainWindow
{
    Q_OBJECT

public:
    open3d_qt(QWidget* parent = Q_NULLPTR);
    bool source_open = false;
    bool target_open = false;
    std::shared_ptr<geometry::PointCloud> source_ori, target_ori;
    std::shared_ptr<geometry::PointCloud> source, target;
    Eigen::Matrix4d matrix_fpfh;
    Eigen::Matrix4d matrix_init;
    Eigen::Matrix4d matrix_icp;
    Eigen::Matrix4d matrix_final;
    double voxel_size;
    int icp_method;
    double icp_threshold_ratio;
    double icp_distance_threshold;
    double icp_iteration;
    int icp_optimation_times;
    double reduce_ratio;
    utility::optional<unsigned int> seed_;
    bool mutual_filter;
    pipelines::registration::RegistrationResult registration_result_fpfh;
    pipelines::registration::RegistrationResult registration_result_icp;
    std::shared_ptr<open3d::visualization::Visualizer> m_visualizer;
    
private slots:
    void on_pushButton_source_clicked();
    void on_pushButton_target_clicked();
    void on_pushButton_fpfh_clicked();
    void on_pushButton_visualize_dowmsampled_clicked();
    void on_pushButton_icp_clicked();
    void on_pushButton_visualize_clicked();
    void on_pushButton_merge_save_clicked();
    void on_pushButton_source_filter_clicked();
    void on_pushButton_target_filter_clicked();

    QString matrix2QString(Eigen::Matrix4d matrix);
    Eigen::Matrix4d String2matrix(std::string);
    void visualizer_reg(std::shared_ptr<open3d::geometry::PointCloud> source_, std::shared_ptr<open3d::geometry::PointCloud> target_, Eigen::Matrix4d matrix_= Eigen::Matrix4d::Identity());
    bool DrawGeometries(const std::vector<std::shared_ptr<const geometry::Geometry>>
        & geometry_ptrs,
        const std::string& window_name = "Open3D",
        int width = 640,
        int height = 480,
        int left = 50,
        int top = 50,
        bool point_show_normal = false,
        bool mesh_show_wireframe = false,
        bool mesh_show_back_face = false,
        Eigen::Vector3d* lookat = nullptr,
        Eigen::Vector3d* up = nullptr,
        Eigen::Vector3d* front = nullptr,
        double* zoom = nullptr);

private:
    Ui::open3d_qtClass ui;
};

void Stringsplit(const std::string& str, const std::string& splits, std::vector<std::string>& res);