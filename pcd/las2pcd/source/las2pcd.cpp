#include <iostream>
#include <cstdlib>
#include <liblas/liblas.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/uniform_sampling.h>
#include "pcl_sample.hpp"
#include "console.hpp"
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


void PrintHelp() {

	std::cout << "Input parameters:" << std::endl
		<< "--path_source path: path to a las pcd need to be convert" << std::endl
		<< "--path_target path: path to save the converted pcd" << std::endl
		<< "--format: save format: binary_compression(0), binary(1), ascii(2), default: 0" << std::endl
		<< "--fix_points: points you want remain after downsample,default 0, it means no downsampling" << std::endl;


}

int main(int argc, char** argv)
{
	/*
	* carlos_lee 20200306
	* add help
	* add target filepath
	*/
	std::string path_source;
	std::string path_target;
	int save_format = 0;
	int fix_points = 0;
	if (argc < 5 || ProgramOptionExistsAny(argc, argv, { "-h", "--help" })) {
		PrintHelp();
		return 0;
	}
	path_source = GetProgramOptionAsString(argc, argv, "--path_source", "");
	path_target = GetProgramOptionAsString(argc, argv, "--path_target", "");
	
	save_format = GetProgramOptionAsInt(argc, argv, "--format", 0);
	fix_points = GetProgramOptionAsInt(argc, argv, "--fix_points", 0);



	std::cout << "Begin to read pcd.\n";
	clock_t start_time = clock();
	std::ifstream ifs(path_source, std::ios::in | std::ios::binary); // 打开las文件
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs); // 读取las文件

	unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();//获取las数据点的个数

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsample(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width = nbPoints;	//保证与las数据点的个数一致	
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	int i = 0;
	uint16_t r1, g1, b1;
	int r2, g2, b2;
	uint32_t rgb;

	while (reader.ReadNextPoint())
	{
		// 获取las数据的x，y，z信息
		cloud->points[i].x = (reader.GetPoint().GetX());
		cloud->points[i].y = (reader.GetPoint().GetY());
		cloud->points[i].z = (reader.GetPoint().GetZ());

		//获取las数据的r，g，b信息
		r1 = (reader.GetPoint().GetColor().GetRed());
		g1 = (reader.GetPoint().GetColor().GetGreen());
		b1 = (reader.GetPoint().GetColor().GetBlue());
		r2 = ceil(((float)r1 / 65536) * (float)256);
		g2 = ceil(((float)g1 / 65536) * (float)256);
		b2 = ceil(((float)b1 / 65536) * (float)256);
		rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);
		cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);

		i++;
	}
	clock_t read_time = clock();
	std::cout << "read " << cloud->size() << " points from file, cost: " << (double)(read_time - start_time) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	if (fix_points != 0) {
		std::cout << "downsampling...\n";
		down_sample_random(cloud, cloud, fix_points);
		clock_t down_sample_time = clock();
		
	}
	
	clock_t write_time_s = clock();
	
	if (save_format == 0) {
		pcl::io::savePCDFileBinaryCompressed(path_target, *cloud);
	}
	else if (save_format == 0) {
		pcl::io::savePCDFileBinary(path_target, *cloud);//存储为pcd类型文件
	}
	else {
		pcl::io::savePCDFileASCII(path_target, *cloud);//存储为pcd类型文件
	}
	clock_t write_time = clock();
	std::cout << "write pcd cost: " << (double)(write_time - write_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	return (0);
}