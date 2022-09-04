#include <iostream>
#include <fstream>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
#define _USE_MATH_DEFINES
#include <math.h>//M_PI
#include <Eigen/Core>

//https ://blog.csdn.net/peach_blossom/article/details/78506184
//由旋转平移矩阵计算旋转角度
void matrix2angle(Eigen::Matrix4d& result_trans, Eigen::Vector3f& result_angle)
{
	double ax, ay, az;
	if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1)
	{
		az = 0;
		double dlta;
		dlta = atan2(result_trans(0, 1), result_trans(0, 2));
		if (result_trans(2, 0) == -1)
		{
			ay = M_PI / 2;
			ax = az + dlta;
		}
		else
		{
			ay = -M_PI / 2;
			ax = -az + dlta;
		}
	}
	else
	{
		ay = -asin(result_trans(2, 0));
		ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
		az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
	}

	result_angle << ax, ay, az;
	result_angle = result_angle / M_PI * 180;

}

void error_r_t(Eigen::Matrix4d& matrix_gt, Eigen::Matrix4d& matrix_compare, Eigen::Vector3f& error_r_angle, Eigen::Vector3f& error_t) {
	
	Eigen::Vector3f angle_gt{0,0,0};
	matrix2angle(matrix_gt, angle_gt);
	Eigen::Vector3f angle_compare{0,0,0};
	matrix2angle(matrix_compare, angle_compare);

	error_r_angle = angle_compare - angle_gt;
	error_r_angle = error_r_angle.array().abs();
	//https://blog.csdn.net/qq_36686437/article/details/108932995
	//translation error x ,y z
	error_t(0) = fabs(matrix_gt(0, 3) - matrix_compare(0, 3));
	error_t(1) = fabs(matrix_gt(1, 3) - matrix_compare(1, 3));
	error_t(2) = fabs(matrix_gt(2, 3) - matrix_compare(2, 3));

}




int main(int argc, char** argv)
{
	/*
	* carlos_lee 20200323
	*/
	std::string matrix_compare_path;
	std::string matrix_gt_path;
	int fix_points = 0;
	if (argc < 2 || argv[1] == "-h" || argv[1] == "--h") {
		std::cout << "input parameters:\n"
			<< "matrix_compare_path\n"
			<< "matrix_gt_path\n"
			<< "\n\nnote:\n"
			<< "1、if you input only one matrix path, the matrix you input will compare with a identity matrix\n"
			<< "2、the matrix in txt must be like this:\n"
			<< "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n" << std::endl;
		return 0;
	}

	matrix_compare_path = argv[1];
	Eigen::Matrix4d matrix_compare = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d matrix_gt = Eigen::Matrix4d::Identity();
	float value = 0;
	std::ifstream fin;
	fin.open(matrix_compare_path, std::ios::in);
	if (!fin.is_open()) {
		std::cout << "failed to open: " << matrix_compare_path << "please check it\n";
		return 0;
	}
	
	for (std::size_t i = 0; i < 4; ++i) {
		for (std::size_t j = 0; j < 4; ++j) {
			fin >> value;
			matrix_compare(i, j) = value;
		}
	}
	fin.close();
	

	if (argc >= 3) {
		matrix_gt_path = argv[2];
		fin.open(matrix_gt_path, std::ios::in);
		if (!fin.is_open()) {
			std::cout << "failed to open: " << matrix_gt_path << "please check it\n";
			return 0;
		}

		for (std::size_t i = 0; i < 4; ++i) {
			for (std::size_t j = 0; j < 4; ++j) {
				fin >> value;
				matrix_gt(i, j) = value;
			}
		}
		fin.close();
	}

	std::cout << "your input:\n"
		<< "matrix_compare:\n" << matrix_compare << std::endl
		<< "matrix_gt:\n" << matrix_gt << std::endl;

	////pcl https://pcl.readthedocs.io/projects/tutorials/en/master/interactive_icp.html#interactive-icp
	//// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	//double theta = M_PI / 4;  // The angle of rotation in radians
	//matrix_compare(0, 0) = std::cos(theta);
	//matrix_compare(0, 1) = -sin(theta);
	//matrix_compare(1, 0) = sin(theta);
	//matrix_compare(1, 1) = std::cos(theta);
	//// A translation on Z axis (0.4 meters)
	//matrix_compare(2, 3) = 0.4;
	////void matrix2angle(Eigen::Matrix4f & result_trans, Eigen::Vector3f & result_angle)


	Eigen::Vector3f error_r_angle{ 0,0,0 };
	Eigen::Vector3f error_t{ 0,0,0 };
	error_r_t(matrix_gt, matrix_compare, error_r_angle, error_t);

	std::cout << "rotation errors(degrees):\n"
		<< "x: " << error_r_angle(0) << "	y: " << error_r_angle(1) << "	z:" << error_r_angle(2) << std::endl
		<< "translation errors:\n"
		<< "x: " << error_t(0) << "	y: " << error_t(1) << "	z:" << error_t(2) << std::endl;


	//if (!pointcloudread_pcd(source_pcd_path, cloud)){
	//	return -1;
	//}
	
	
	
	//clock_t write_time_s = clock();
	////
	//clock_t write_time = clock();
	//std::cout << "write pcd cost: " << (double)(write_time - write_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	

	return (0);
}