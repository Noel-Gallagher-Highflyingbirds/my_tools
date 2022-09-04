#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/time.h>   // TicToc 
#include "pcl_sample.hpp"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int pointcloudread_pcd(std::string& filename, PointCloudT::Ptr& cloud)
{
	pcl::console::TicToc timecount;
	timecount.tic();
	if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1)//*打开点云文件
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return false;
	}
	//pcl::io::savePCDFileASCII("point_source/save/output/1127cloud_maoxian_statisticalremoval-k100-4-ASCII.pcd", *cloud_in_tgt);

	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from pcd"
		<< std::endl;
	std::cout << "\nLoaded file " << filename << " (" << cloud->size() << " points) in " << timecount.toc() << " ms\n" << std::endl;
	return true;
}


bool pointcloudread_ply(std::string& filename, PointCloudT::Ptr& cloud)
{
	pcl::console::TicToc timecount;
	timecount.tic();
	if (pcl::io::loadPLYFile<PointT>(filename, *cloud) == -1)//*打开点云文件
	{
		PCL_ERROR("Couldn't read file ply\n");
		return false;
	}
	//pcl::io::savePCDFileASCII("point_source/save/output/1127cloud_maoxian_statisticalremoval-k100-4-ASCII.pcd", *cloud_in_tgt);

	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from ply"
		<< std::endl;
	std::cout << "\nLoaded file " << filename << " (" << cloud->size() << " points) in " << timecount.toc() << " ms\n" << std::endl;
	return true;
}

int main(int argc,char** argv) {
	PointCloudT::Ptr pcd_source(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr pcd_downsample(new PointCloudT);  // Original point cloud
	std::string source_pcd_path;
	std::string target_pcd_path = "./pcd_downsample.pcd";
	int downsample_mode = 0;
	float voxel_size = 2.0;
	int fix_points = 2000;

	if (argc < 2 || argv[1] == "-h" || argv[1] == "--h") {
		std::cout << "input 5 parameters:\n"
			<< "source point cloud path.\n"
			<< "point cloud path to save downsample pcd. defalut: " << target_pcd_path << std::endl
			<< "downsample mode: 0(voxel_ori), 1(voxel_improved), 2(uniform), 3(random), default: " << downsample_mode << std::endl
			<< "voxelsize, for downsample mode 0 1 2，default: " << voxel_size << std::endl
			<< "fix number to downsample, for downsample mode 1, 3,default: " << fix_points << std::endl
			<< "when choose mode 0 2, fix number can be set arbitrarily, when choose mode 3, voxelsize can be set arbitrarily.\n";

	}
	source_pcd_path = argv[1];
	if (argc >= 3) {
		target_pcd_path = argv[2];
	}
	if (argc >= 4) {
		downsample_mode = atoi(std::string(argv[3]).c_str());
	}
	if (argc >= 5) {
		voxel_size = atof(std::string(argv[4]).c_str());
	}
	if (argc >= 6) {
		fix_points = atoi(std::string(argv[5]).c_str());
	}
	std::cout << "Use parameters:\n"
		<< "source_pcd_path: " << source_pcd_path << std::endl
		<< "target_pcd_path: " << target_pcd_path << std::endl
		<< "downsample_mode: " << downsample_mode << std::endl
		<< "voxel_size: " << voxel_size << std::endl
		<< "fix_points: " << fix_points << std::endl;
	clock_t read_time_s = clock();
	if (!pointcloudread_pcd(source_pcd_path, pcd_source)) {
		return 0;
	}
	clock_t read_time = clock();
	std::cout << "read pcd cost: " << (double)(read_time - read_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	//downsample
	clock_t downsample_time_s = clock();
	switch (downsample_mode) {
	case 0:
		down_sample_voxel(pcd_source, pcd_downsample, voxel_size, false);
		pcl::io::savePCDFileBinaryCompressed(target_pcd_path, *pcd_downsample);
		break;
	case 1:
		down_sample_voxel(pcd_source, pcd_downsample, voxel_size, true);
		pcl::io::savePCDFileBinaryCompressed(target_pcd_path, *pcd_downsample);
		break;
	case 2:
		down_sample_uniform(pcd_source, pcd_downsample, voxel_size);
		pcl::io::savePCDFileBinaryCompressed(target_pcd_path, *pcd_downsample);
		break;
	case 3:
		down_sample_random(pcd_source, pcd_downsample, fix_points);
		pcl::io::savePCDFileBinaryCompressed(target_pcd_path, *pcd_downsample);
		break;
	default:
		std::cout << "do nothing\n";
		break;

	}
	clock_t downsample_time = clock();
	std::cout << "downsample pcd cost: " << (double)(downsample_time - downsample_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	return 0;


}