#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "console.hpp"

typedef pcl::PointXYZRGB PointT;
// typedef pcl::PointXYZI PointT;

typedef pcl::PointCloud<PointT> PointCloudT;

void PrintHelp() {

	std::cout << "Input parameters:" << std::endl
		<< "--path_source path: path to a pcd need to be convert" << std::endl
		<< "--path_target path: path to save the converted pcd" << std::endl
		<< "--scale: a scale factor will apply on pcd, default: 1.0" << std::endl
		<< "--pcd_type(no use now): PointCloudXYZRGB(0), PointCloudXYZI(1), default: 0" << std::endl
		<< "--format: save format: ascii(0), binary(1), ascii(2), default: 1" << std::endl;;
}


bool pointcloudread_pcd(std::string& filename, PointCloudT::Ptr& cloud)
{
	clock_t read_time_s = clock();

	if(filename.find(".pcd")!=-1){
	pcl::PCDReader reader;
	if (reader.read(filename,*cloud)== -1){
			PCL_ERROR("Couldn't read file:\n");
			PCL_ERROR(filename.c_str());
			return(false);
		}
	}
	if(filename.find(".ply")!=-1){
		pcl::PLYReader reader;
		if (reader.read(filename,*cloud)== -1){
		PCL_ERROR("Couldn't read file:\n");
		PCL_ERROR(filename.c_str());
		return(false);
		}
	}


	std::cout << "Loaded "
		<< cloud->points.size()
		<< " data points from file."
		<< std::endl;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	std::cout << "after remove NAN remain " << cloud->points.size() << std::endl;

	clock_t read_time = clock();
	std::cout << "\nLoaded file " << filename << " (" << cloud->size() << " points) in "
		<< (double)(read_time - read_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	return true;
}


int main(int argc, char** argv)
{
	/*
	* carlos_lee 202207
	*/

	std::string path_source;
	std::string path_target;
	int save_format = 1;
	int fix_points = 0;
	float scale = 1.0;
	int pcd_type = 0;
	if (argc < 3 || argv[1] == "-h" || argv[1] == "--h") {
		PrintHelp();
		return 0;
	}
	path_source = GetProgramOptionAsString(argc, argv, "--path_source", "");
	path_target = GetProgramOptionAsString(argc, argv, "--path_target", "");
	if(path_source.find(".pcd")==-1 && path_source.find(".ply")==-1){
		PCL_ERROR("input --path_source  with extension .pcd or .ply\n");
		return 0;
	}
	if(path_target.find(".pcd")==-1 && path_target.find(".ply")==-1){
		PCL_ERROR("input --path_target  with extension .pcd or .ply\n");
		return 0;
	}

	save_format = GetProgramOptionAsInt(argc, argv, "--format", 1);
	scale = GetProgramOptionAsDouble(argc, argv, "--scale", 1.0);
	//pcd_type = GetProgramOptionAsInt(argc, argv, "--pcd_type", 0);


	std::cout << "parameters:\n"
		<< "source point cloud path: " << path_source << std::endl
		<< "point cloud path to save pcd: " << path_target << std::endl
		<< "scale: " << scale << std::endl
		<< "pcd_type: " << pcd_type << std::endl
		<< "format to save: " << ((save_format == 0) ? "ascii" : "binary") << std::endl;


	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	if (!pointcloudread_pcd(path_source, cloud)){
		return -1;
	}
	for (auto& point : cloud->points) {
		point.x = point.x * scale;
		point.y = point.y * scale;
		point.z = point.z * scale;
	}
	clock_t write_time_s = clock();

	if(path_target.find(".pcd")!=-1){
	pcl::PCDWriter writer;
	std::cout<<"the converted pcd has saved to "<<path_target<<std::endl;
	writer.write<PointT>(path_target,*cloud,save_format);
	}
	if(path_target.find(".ply")!=-1){
	pcl::PLYWriter writer;
	std::cout<<"the converted pcd has saved to "<<path_target<<std::endl;
	writer.write<PointT>(path_target,*cloud,save_format);
	}

	clock_t write_time = clock();
	std::cout << "write pcd cost: " << (double)(write_time - write_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	return (0);
}


