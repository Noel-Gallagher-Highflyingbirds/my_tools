#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "console.hpp"
typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZI PointTI;

typedef pcl::PointCloud<PointT> PointCloudT;

void PrintHelp() {

	std::cout << "Input parameters:" << std::endl
		<< "--path_source path: path to a las pcd need to be scale" << std::endl
		<< "--path_target path: path to save the converted pcd" << std::endl
		<< "--scale: a scale factor will apply on pcd, default: 1.0" << std::endl
		<< "--pcd_type(no use now): PointCloudXYZRGB(0), PointCloudXYZI(1), default: 0" << std::endl
		<< "--format: save format: binary_compression(0), binary(1), ascii(2), default: 0" << std::endl;;
}


bool pointcloudread_pcd(std::string& filename, PointCloudT::Ptr& cloud)
{
	clock_t read_time_s = clock();
	if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1)//*open ply file
	{
		PCL_ERROR("Couldn't read file:\n");
		PCL_ERROR(filename.c_str());
		return(false);
	}

	std::cout << "Loaded "
		<< cloud->points.size()
		<< " data points from file."
		<< std::endl;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	std::cout << "after remove NAN，remain " << cloud->points.size() << std::endl;

	clock_t read_time = clock();
	std::cout << "\nLoaded file " << filename << " (" << cloud->size() << " points) in "
		<< (double)(read_time - read_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	return true;
}


int main(int argc, char** argv)
{
	/*
	* carlos_lee 20200313
	*/
	std::string path_source;
	std::string path_target;
	int save_format = 0;
	int fix_points = 0;
	float scale = 1.0;
	int pcd_type = 0;
	if (argc < 4 || argv[1] == "-h" || argv[1] == "--h") {
		PrintHelp();
		return 0;
	}
	path_source = GetProgramOptionAsString(argc, argv, "--path_source", "");
	path_target = GetProgramOptionAsString(argc, argv, "--path_target", "");

	save_format = GetProgramOptionAsInt(argc, argv, "--format", 0);
	scale = GetProgramOptionAsDouble(argc, argv, "--scale", 1.0);
	//pcd_type = GetProgramOptionAsInt(argc, argv, "--pcd_type", 0);


	std::cout << "parameters:\n"
		<< "source point cloud path: " << path_source << std::endl
		<< "point cloud path to save pcd: " << path_target << std::endl
		<< "scale: " << scale << std::endl
		<< "pcd_type: " << pcd_type << std::endl
		<< "format to save: " << ((save_format == 0) ? "binary" : "ascii") << std::endl;


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(new pcl::PointCloud<pcl::PointXYZI>);
	//if (pcd_type == 0) {
	//	cloud = cloud_rgb;
	//}
	//else if (pcd_type == 1) {
	//	cloud = cloud_intensity;
	//}
	////add pcd_type...
	////else if (pcd_type == xxx) {
	////	cloud = xxxx;
	////}

	//else {//error
	//	PCL_ERROR("pcd_type: PointCloudXYZRGB(0), PointCloudXYZI(1), default: 0\n");
	//}

	if (!pointcloudread_pcd(path_source, cloud_rgb)){
		return -1;
	}
	for (auto& point : cloud_rgb->points) {
		point.x = point.x * scale;
		point.y = point.y * scale;
		point.z = point.z * scale;
	}
	clock_t write_time_s = clock();
	
	if (save_format == 0) {
		pcl::io::savePCDFileBinary(path_target, *cloud_rgb);//存储为pcd类型文件
	}
	else {
		pcl::io::savePCDFileASCII(path_target, *cloud_rgb);//存储为pcd类型文件
	}
	clock_t write_time = clock();
	std::cout << "write pcd cost: " << (double)(write_time - write_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	return (0);
}