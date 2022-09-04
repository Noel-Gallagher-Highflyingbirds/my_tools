#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include "pcl_sample.hpp"
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


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
	std::string source_pcd_path;
	std::string target_pcd_path;
	int save_format = 0;
	int fix_points = 0;
	if (argc < 3 || argv[1] == "-h" || argv[1] == "--h") {
		std::cout << "input parameters:\n"
			<< "source point cloud path:xxx.ply\n"
			<< "point cloud path to save pcd.\n"
			<< "format to save: binary(0), ascii(1), default: " << save_format << std::endl
			<< "fix_points: points you want remain after downsample,default 0, it means no downsampling.\n";
		return 0;

	}
	
	if (argc >= 2) {
		source_pcd_path = argv[1];
	}	
	if (argc >= 3) {
		target_pcd_path = argv[2];
	}
	if (argc >= 4) {
		save_format = atoi(std::string(argv[3]).c_str());
	}
	if (argc >= 5) {
		fix_points = atoi(std::string(argv[4]).c_str());
	}
	std::cout << "parameters:\n"
		<< "source point cloud path: " << argv[1] << std::endl
		<< "point cloud path to save pcd: " << target_pcd_path << std::endl
		<< "format to save: " << ((save_format == 0) ? "binary" : "ascii") << std::endl
		<< "fix_points: " << fix_points << std::endl;


	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	if (!pointcloudread_pcd(source_pcd_path, cloud)){
		return -1;
	}

	if (fix_points != 0) {
		std::cout << "downsampling...\n";
		down_sample_random(cloud, cloud, fix_points);
		
	}
	
	clock_t write_time_s = clock();
	
	if (save_format == 0) {
		pcl::io::savePLYFileBinary(target_pcd_path, *cloud);//存储为pcd类型文件
	}
	else {
		pcl::io::savePLYFileASCII(target_pcd_path, *cloud);//存储为pcd类型文件
	}
	clock_t write_time = clock();
	std::cout << "write pcd cost: " << (double)(write_time - write_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	return (0);
}