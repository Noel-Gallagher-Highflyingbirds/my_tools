#include <iostream>
#include <fstream>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
#define _USE_MATH_DEFINES
#include <math.h>//M_PI
#include <Eigen/Core>



int main(int argc, char** argv)
{
	/*
	* carlos_lee 20200323
	* reference: https://zhuanlan.zhihu.com/p/45404840
	*/
	std::string matrix_path;
	int fix_points = 0;
	if (argc < 9 || argv[1] == "-h" || argv[1] == "--h") {
		std::cout << "input parameters need:\n"
			<<"7 parameters:	translation:t_x,t_y,t_z		Quaternion:x,y,z,w\n"
			<<"path to save matrix()\n"
			<< "note:\n"
			<<"7 parameters like: 0 0 0 0 0 0 1\n" 
			<<"path to save matrix like ./matrix_p7.txt\n"
			<< std::endl;
		return 0;
	}

	double t_x = atof(argv[1]);
	double t_y = atof(argv[2]);
	double t_z = atof(argv[3]);
	double z = atof(argv[4]);
	double y = atof(argv[5]);
	double x = atof(argv[6]);
	double w = atof(argv[7]);
	matrix_path = argv[8];
	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
	matrix(0, 0) = 1 - 2 * y * y - 2 * z * z;
	matrix(0, 1) = 2 * x * y - 2 * z * w;
	matrix(0, 2) = 2 * x * z + 2 * y * w;
	matrix(0, 3) = t_x;

	matrix(1, 0) = 2 * x * y + 2 * y * z;
	matrix(1, 1) = 1 - 2 * x * x - 2 * z * z;
	matrix(1, 2) = 2 * y * z - 2 * x * w;
	matrix(1, 3) = t_y;

	matrix(2, 0) = 2 * x * z - 2 * y * w;
	matrix(2, 1) = 2 * y * z + 2 * x * w;
	matrix(2, 2) = 1 - 2 * x * x - 2 * y * y;
	matrix(2, 3) = t_z;

	matrix(3, 0) = 0;
	matrix(3, 1) = 0;
	matrix(3, 2) = 0;
	matrix(3, 3) = 1;


	std::ofstream fout(matrix_path, std::ios::out);
	if (!fout.is_open()) {
		std::cout << "failed to open: " << matrix_path << "please check it\n";
		return 0;
	}
	float value = 0;
	for (std::size_t i = 0; i < 4; ++i) {
		for (std::size_t j = 0; j < 4; ++j) {
			fout << matrix(i, j)<<" ";
		}
		fout << "\n";
	}

	fout << "7 parameters:\n"
		<< "t_x	t_y	t_z	x	y	z	w\n"
		<< t_x << "	" << t_y << "	" << t_z << "	"
		<< x << "	" << y << "	" << z << "	" << w << std::endl;

	fout.close();
	

	return (0);
}