#include "registration_qt.h"
#include <QFile>
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include "registration_tools.h"
#include "open3d/Open3D.h"

//#include "ui_open3d_qt.h"
/*
* ��vs2013��ʹ��Qt�޷�����cout��cin��exe�����������
https://blog.csdn.net/Liuqz2009/article/details/107935906
qlineEdit
https://wenku.baidu.com/view/5fdf4acc9a8fcc22bcd126fff705cc1754275f72.html


--path_source D:\carlos\my_tools\pcd\code\open3d_test\data\08_33_32_128.8741_101cloudPoind.pcd 
--path_target D:\carlos\my_tools\pcd\code\open3d_test\data\08_31_43_128.8606_101cloudPoind.pcd 
--voxel_size 2 --use_icp 50 --fix_seed 123456 --icp_method 1 --icp_threshold_ratio 1.5
*/

open3d_qt::open3d_qt(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    this->matrix_fpfh = Eigen::Matrix4d::Identity();
    this->matrix_init = Eigen::Matrix4d::Identity();
    this->matrix_icp = Eigen::Matrix4d::Identity();
    this->matrix_final = Eigen::Matrix4d::Identity();
    this->icp_threshold_ratio = 1.5;
    this->seed_ = utility::nullopt;
    this->mutual_filter = true;
    this->icp_iteration = 30;
    this->icp_optimation_times = 3;
    this->reduce_ratio = 0.8;
    
}
void open3d_qt::on_pushButton_source_clicked() {
    

    //https://blog.csdn.net/qq_38400517/article/details/78897446
    QString fileName;
    fileName = QFileDialog::getOpenFileName(this, tr("select a source pointcloud file"), "", "*.*");
    std::string file_name = fileName.toStdString();
    if (!fileName.isNull()) {
        std::shared_ptr<geometry::PointCloud> pcd(new geometry::PointCloud);
        if (!open3d::io::ReadPointCloud(file_name, *pcd)) {
            QMessageBox::warning(this, "Error", "read file failed, please check your pointcloud file path");
            return;
        }
        std::cout<<"source: "<<file_name<<std::endl;
        source_ori = pcd;
        //source_open = true;
        this->matrix_fpfh = Eigen::Matrix4d::Identity();
        this->matrix_init = Eigen::Matrix4d::Identity();
        this->matrix_icp = Eigen::Matrix4d::Identity();
        this->matrix_final = Eigen::Matrix4d::Identity();
        ui.textEdit_matrix_init->clear();
        ui.textEdit_matrix_icp->clear();
        ui.textEdit_matrix_final->clear();
        this->source = nullptr;
    }
}

void open3d_qt::on_pushButton_target_clicked() {

    QString fileName;
    fileName = QFileDialog::getOpenFileName(this, tr("select a target pointcloud file"), "", "*.*");
    std::string file_name = fileName.toStdString();
    if (!fileName.isNull()) {
        std::shared_ptr<geometry::PointCloud> pcd(new geometry::PointCloud);
        if (!open3d::io::ReadPointCloud(file_name, *pcd)) {
            QMessageBox::warning(this, "Error", "read file failed, please check your pointcloud file path");
            return;
        }
        std::cout<<"target: "<<file_name<<std::endl;
        target_ori = pcd;
        //target_open = true;
        this->matrix_fpfh = Eigen::Matrix4d::Identity();
        this->matrix_init = Eigen::Matrix4d::Identity();
        this->matrix_icp = Eigen::Matrix4d::Identity();
        this->matrix_final = Eigen::Matrix4d::Identity();
        ui.textEdit_matrix_init->clear();
        ui.textEdit_matrix_icp->clear();
        ui.textEdit_matrix_final->clear();
        this->target = nullptr;
    }

}

void open3d_qt::on_pushButton_visualize_clicked() {
    if (target_ori != nullptr && source_ori != nullptr) {
        open3d_qt::visualizer_reg(source_ori, target_ori);
    }
    else {
        QMessageBox::warning(this, "Error", "please choose source pointcloud and target pointcloud");
    }
}
void open3d_qt::on_pushButton_visualize_dowmsampled_clicked() {
    if (source_ori == nullptr || target_ori == nullptr) {
        std::cout << "please choose source pointcloud and target pointcloud" << std::endl;
        QMessageBox::warning(this, "Error", "please choose source pointcloud and target pointcloud");
        return;
    }
    open3d_qt::voxel_size = ui.lineEdit_voxel_size->text().toDouble();
    //������
    std::shared_ptr<geometry::PointCloud> pcd_down_source,pcd_down_target;
    pcd_down_source = source_ori->VoxelDownSample(voxel_size);
    //std::cout <<"\n\n\n\npcd_down_source:"<< pcd_down_source->points_.size() << std::endl;
    //QMessageBox::information(this, "downsample", std::to_string(pcd_down_source->points_.size()).c_str());
    pcd_down_target = target_ori->VoxelDownSample(voxel_size);
    //open3d_qt::visualizer_reg(source_ori, target_ori);
    open3d_qt::visualizer_reg(pcd_down_source, pcd_down_target);
}

void open3d_qt::on_pushButton_fpfh_clicked() {

    if (source_ori == nullptr || target_ori == nullptr) {
        std::cout << "please choose source pointcloud and target pointcloud" << std::endl;
        QMessageBox::warning(this, "Error", "please choose source pointcloud and target pointcloud");
        return;
    }
    // if (matrix_fpfh != Eigen::Matrix4d::Identity()) {
    //     open3d_qt::visualizer_reg(source_ori, target_ori, matrix_fpfh);
    //     return;
    // }
    //QTextStream in(&file);
    //QApplication::setOverrideCursor(Qt::WaitCursor);
    open3d_qt::voxel_size = ui.lineEdit_voxel_size->text().toDouble();
    open3d_qt::seed_ = 123456;

    std::shared_ptr<pipelines::registration::Feature> source_fpfh, target_fpfh;
    std::tie(source, source_fpfh) = PreprocessPointCloud(source_ori, voxel_size);
    std::tie(target, target_fpfh) = PreprocessPointCloud(target_ori, voxel_size);
    open3d_qt::registration_result_fpfh = fpfh_registration(source, target, source_fpfh, target_fpfh, voxel_size, seed_, mutual_filter);
    // std::cout << "*****coarse registration result*****\n";
    // std::cout << std::endl
    //     << "fpfh matrix:" << std::endl
    //     << registration_result_fpfh.transformation_ << std::endl;
    // std::cout << "inlier(correspondence_set size):"
    //     << registration_result_fpfh.correspondence_set_.size() << std::endl;
    // std::cout << "\nfitness_(For RANSAC: inlier ratio (# of inlier correspondences / # of all correspondences)): " << registration_result_fpfh.fitness_ << std::endl
    //     << "RMSE of all inlier correspondences: " << registration_result_fpfh.inlier_rmse_ << std::endl;

    matrix_init = matrix_fpfh = registration_result_fpfh.transformation_;
    ui.textEdit_matrix_init->clear();
    ui.textEdit_matrix_init->setText(open3d_qt::matrix2QString(registration_result_fpfh.transformation_));

    //open3d_qt::visualizer_reg(source_ori, target_ori);
    open3d_qt::visualizer_reg(source_ori, target_ori, matrix_init);

}
void open3d_qt::on_pushButton_icp_clicked() {
    if (source_ori == nullptr || target_ori == nullptr) {
        std::cout << "please choose source pointcloud and target pointcloud" << std::endl;
        QMessageBox::warning(this, "Error", "please choose source pointcloud and target pointcloud");
        return;
    }

    open3d_qt::voxel_size = ui.lineEdit_voxel_size->text().toDouble();

    if (source == nullptr) {
        //������
        source = source_ori->VoxelDownSample(voxel_size);
        std::cout << "voxel_size=" << voxel_size << ", after downsample "
            << source->points_.size() << "points left" << std::endl;

        //���㷨����
        clock_t EstimateNormals_time_s = clock();
        source->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2, 30));
        clock_t EstimateNormals_time = clock();
        std::cout << "EstimateNormals costs: " << (double)(EstimateNormals_time - EstimateNormals_time_s) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;

        //ָ������������
        source->OrientNormalsToAlignWithDirection();
    }
    if(target == nullptr){

        target = target_ori->VoxelDownSample(voxel_size);
        std::cout << "voxel_size=" << voxel_size << ", after downsample "
            << target->points_.size() << "points left" << std::endl;
        //���㷨����
        clock_t EstimateNormals_time_s = clock();
        target->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2, 30));
        clock_t EstimateNormals_time = clock();
        std::cout << "EstimateNormals costs: " << (double)(EstimateNormals_time - EstimateNormals_time_s) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;

        //ָ������������
        target->OrientNormalsToAlignWithDirection();

    }
    //if (matrix_final != Eigen::Matrix4d::Identity()) {
    //    open3d_qt::visualizer_reg(source_ori, target_ori, matrix_final);
    //    return;
    //}
    open3d_qt::icp_distance_threshold = voxel_size * open3d_qt::icp_threshold_ratio;
    open3d_qt::icp_method = ui.comboBox_icp_method->currentIndex();
    std::cout << "ui.comboBox_icp_method->currentIndex(): " << icp_method << std::endl;
    open3d_qt::icp_iteration = ui.lineEdit_icp_iteration->text().toInt();



    QString matrix_init_qstr=ui.textEdit_matrix_init->document()->toPlainText();
    std::string matrix_init_str=matrix_init_qstr.toStdString();
    if(matrix_init_str!=""){
        matrix_init=open3d_qt::String2matrix(matrix_init_str);
        std::cout<<"matrix_init\n"<<matrix_init<<std::endl;
    }else{
        std::cout<<"no initial matrix, use Identity matrix\n"<<std::endl;
        ui.textEdit_matrix_init->clear();
        matrix_init=Eigen::Matrix4d::Identity();
        ui.textEdit_matrix_init->setText(open3d_qt::matrix2QString(matrix_init));
    }

    
    open3d_qt::icp_optimation_times = ui.lineEdit_icp_optimation_times->text().toInt();
    open3d_qt::reduce_ratio = ui.lineEdit_reduce_ratio->text().toDouble();
    for (std::size_t icp_i = 0; icp_i < open3d_qt::icp_optimation_times; ++icp_i) {
        std::cout << "this is the "<<icp_i+1<<"th icp optimation, and use" << icp_iteration << " iteration every optimation" << std::endl;
        std::cout << "courrent icp_distance_threshold: " << icp_distance_threshold << std::endl;
        //icp registration
        registration_result_icp = icp_registration(source, target, icp_distance_threshold, matrix_icp * matrix_init, icp_method, icp_iteration);
        
        matrix_icp = registration_result_icp.transformation_*matrix_icp;
        ui.textEdit_matrix_icp->clear();
        ui.textEdit_matrix_icp->setText(open3d_qt::matrix2QString(matrix_icp));
        icp_distance_threshold = reduce_ratio * icp_distance_threshold;
    }
    icp_distance_threshold = voxel_size * open3d_qt::icp_threshold_ratio;
    std::cout << "******total icp optimation matrix******\n" <<
        matrix_icp << std::endl;

    matrix_final = matrix_icp * matrix_init;
    matrix_icp = Eigen::Matrix4d::Identity();
    // std::cout << "*****fpfh_icp_result*****\n";
    // std::cout << std::endl
    //     << "icp based on fpfh matrix:" << std::endl
    //     << registration_result_icp.transformation_ << std::endl;
    // std::cout << "inlier(correspondence_set size):"
    //     << registration_result_icp.correspondence_set_.size() <<
    //     " all(pcd size after voxel downsample): " << source->points_.size() << std::endl;

    // std::cout << "\nfitness_(# of inlier correspondences / # of points in target): " << registration_result_icp.fitness_ << std::endl
    //     << "RMSE of all inlier correspondences: " << registration_result_icp.inlier_rmse_ << std::endl;
    // std::cout << std::endl
    // << "fpfh + icp registration matrix:" << std::endl
    // << matrix_final << std::endl;

    ui.textEdit_matrix_final->clear();
    ui.textEdit_matrix_final->setText(open3d_qt::matrix2QString(matrix_final));


    open3d_qt::visualizer_reg(source_ori, target_ori, matrix_final);
    
}



QString open3d_qt::matrix2QString(Eigen::Matrix4d matrix) {
    QString matrix_s;
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            matrix_s.append(QString::number(matrix(i, j)));
            matrix_s.append(' ');
        }
        matrix_s.append('\n');
    }
    return matrix_s;
}
Eigen::Matrix4d open3d_qt::String2matrix(std::string str) {
    std::vector<std::string> str_split;
    Stringsplit(str,"\n",str_split);
    Eigen::Matrix4d matrix=Eigen::Matrix4d::Identity();

    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            std::vector<std::string> str_split_2;
            Stringsplit(str_split[i]," ",str_split_2);
            matrix(i,j)=atof(str_split_2[j].c_str());
        }
    }
    return matrix;
}


void open3d_qt::visualizer_reg(std::shared_ptr<open3d::geometry::PointCloud> source_, std::shared_ptr<open3d::geometry::PointCloud> target_, Eigen::Matrix4d matrix_) {


    target_->PaintUniformColor({ 1, 0, 0 });                  //��
    source_->PaintUniformColor({ 0, 1, 0 });                  //��


    if (matrix_ == Eigen::Matrix4d::Identity()) {
        //visualization::DrawGeometries(
        DrawGeometries(
            { source_, target_ },
            "origin pcd", 960, 900, 960, 100);
    }
    else {
        std::shared_ptr<geometry::PointCloud> source_transformed(
            new geometry::PointCloud);
        *source_transformed = *source_ori;
        source_transformed->PaintUniformColor({ 0, 0, 1 });  //��
        source_transformed->Transform(matrix_);

        //visualization::DrawGeometries(
        DrawGeometries(
            { target_ ,source_transformed },
            "registration(blue)", 960, 900, 960, 100);
    }
}



bool open3d_qt::DrawGeometries(const std::vector<std::shared_ptr<const geometry::Geometry>>
    & geometry_ptrs,
    const std::string& window_name /* = "Open3D"*/,
    int width /* = 640*/,
    int height /* = 480*/,
    int left /* = 50*/,
    int top /* = 50*/,
    bool point_show_normal /* = false */,
    bool mesh_show_wireframe /* = false */,
    bool mesh_show_back_face /* = false */,
    Eigen::Vector3d* lookat /* = nullptr */,
    Eigen::Vector3d* up /* = nullptr */,
    Eigen::Vector3d* front /* = nullptr */,
    double* zoom /* = zoom */) {

    if (m_visualizer != nullptr) {
        int sss=m_visualizer.use_count();
        m_visualizer->DestroyVisualizerWindow();
        m_visualizer == nullptr;
    }

    std::shared_ptr<open3d::visualization::Visualizer> visualizer(new open3d::visualization::Visualizer);
    m_visualizer = visualizer;
    if (!visualizer->CreateVisualizerWindow(window_name, width, height, left,
        top)) {
        utility::LogWarning(
            "[DrawGeometries] Failed creating OpenGL "
            "window.");
        return false;
    }
    visualizer->GetRenderOption().point_show_normal_ = point_show_normal;
    visualizer->GetRenderOption().mesh_show_wireframe_ = mesh_show_wireframe;
    visualizer->GetRenderOption().mesh_show_back_face_ = mesh_show_back_face;
    for (const auto& geometry_ptr : geometry_ptrs) {
        if (!visualizer->AddGeometry(geometry_ptr)) {
            utility::LogWarning("[DrawGeometries] Failed adding geometry.");
            utility::LogWarning(
                "[DrawGeometries] Possibly due to bad geometry or wrong"
                " geometry type.");
            return false;
        }
    }

    open3d::visualization::ViewControl& view_control = visualizer->GetViewControl();
    if (lookat != nullptr) {
        view_control.SetLookat(*lookat);
    }
    if (up != nullptr) {
        view_control.SetUp(*up);
    }
    if (front != nullptr) {
        view_control.SetFront(*front);
    }
    if (zoom != nullptr) {
        view_control.SetZoom(*zoom);
    }

    visualizer->Run();
    visualizer->DestroyVisualizerWindow();
    m_visualizer = nullptr;
    return true;
}

// 使用字符串分割
void Stringsplit(const std::string& str, const std::string& splits, std::vector<std::string>& res)
{
	if (str == "")		return;
	//在字符串末尾也加入分隔符，方便截取最后一段
	std::string strs = str + splits;
	size_t pos = strs.find(splits);
	int step = splits.size();

	// 若找不到内容则字符串搜索函数返回 npos
	while (pos != strs.npos)
	{
		std::string temp = strs.substr(0, pos);
		res.push_back(temp);
		//去掉已分割的字符串,在剩下的字符串中进行分割
		strs = strs.substr(pos + step, strs.size());
		pos = strs.find(splits);
	}
}

void open3d_qt::on_pushButton_merge_save_clicked(){
    
    std::cout<<"save pcd to"<<std::endl;
    std::shared_ptr<geometry::PointCloud> pcd_save(
        new geometry::PointCloud);
    
    if (source_ori != nullptr) {
        matrix_final = matrix_icp * matrix_init;
        *pcd_save = *source_ori;
        pcd_save->Transform(matrix_final);
        if(target_ori != nullptr){
        *pcd_save=*pcd_save+*target_ori;
        }
        open3d_qt::visualizer_reg(pcd_save, pcd_save);

        QString strFileName = QFileDialog::getSaveFileName(this,tr("Save PCD"),  //类函数QFileDiaLog:获取文件路径//getSaveFileName:获取保存文件名字
                                            "merged.pcd",
                                            "PCD(*.pcd);;PLY(*.ply)");
        if(!strFileName.isNull()){
            std::string file_name = strFileName.toStdString();
            std::cout<<file_name<<std::endl;
            open3d::io::WritePointCloud(file_name, *pcd_save);
        }
        else{
            QMessageBox::warning(this, "Error", "please choose path to save pcd");

        }
    }
    else {
        QMessageBox::warning(this, "Error", "please choose source pointcloud and target pointcloud");
    }


}

void open3d_qt::on_pushButton_source_filter_clicked(){
    std::cout<<"on_pushButton_source_filter_clicked"<<std::endl;
    if (source_ori == nullptr) {
    std::cout << "please choose source pointcloud" << std::endl;
    QMessageBox::warning(this, "Error", "please choose source pointcloud");
    return;
    }
    int neighbors=ui.lineEdit_nb_neighbors->text().toInt();
    double std_ratio=ui.lineEdit_std_ratio->text().toDouble();
    std::cout<<"neighbors: "<<neighbors<<" std_ratio: "<<std_ratio<<std::endl;
    std::cout<<"before filter: "<<source_ori->points_.size()<<" points"<<std::endl;

    auto sta_filered=source_ori->RemoveStatisticalOutliers(neighbors,std_ratio);
    source_ori=std::get<0>(sta_filered);
    std::cout<<"after filter: "<<source_ori->points_.size()<<" points"<<std::endl;
    DrawGeometries(
            { source_ori },
            "source_ori", 960, 900, 960, 100);



}
void open3d_qt::on_pushButton_target_filter_clicked(){
    std::cout<<"on_pushButton_target_filter_clicked"<<std::endl;
    if (target_ori == nullptr) {
    std::cout << "please choose source pointcloud" << std::endl;
    QMessageBox::warning(this, "Error", "please choose target pointcloud");
    return;
    }
    int neighbors=ui.lineEdit_nb_neighbors->text().toInt();
    double std_ratio=ui.lineEdit_std_ratio->text().toDouble();
    std::cout<<"before filter: "<<target_ori->points_.size()<<" points"<<std::endl;

    auto sta_filered=target_ori->RemoveStatisticalOutliers(neighbors,std_ratio);
    target_ori=std::get<0>(sta_filered);
    std::cout<<"after filter: "<<target_ori->points_.size()<<" points"<<std::endl;
    DrawGeometries(
            { target_ori },
            "source_ori", 960, 900, 960, 100);

}