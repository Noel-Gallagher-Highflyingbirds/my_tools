#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/time.h>   // TicToc 
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "console.hpp"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void PrintHelp() {

	std::cout << "Input parameters:" << std::endl
		<< "--path_source path: path to a pcd need to be filter" << std::endl
		<< "--path_target path: path to save the filtered pcd" << std::endl
		<< "--MeanK: number of neighbors around the target point,defualt: 50" << std::endl
		<< "--StddevMulThresh: standard deviation, defualt: 2.0" << std::endl
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


int main(int argc,char** argv) {
	PointCloudT::Ptr pcd_source(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr pcd_filtered(new PointCloudT);  // Original point cloud
	int save_format=1;
	std::string path_source;
	std::string path_target;
	int MeanK = 50;
	float StddevMulThresh = 2.0;

	if (argc < 3 || argv[1] == "-h" || argv[1] == "--h") {
		PrintHelp();
		return 0;
	}
	path_source = GetProgramOptionAsString(argc, argv, "--path_source", "");
	path_target = GetProgramOptionAsString(argc, argv, "--path_target", "");
	MeanK = GetProgramOptionAsInt(argc, argv, "--MeanK", 50);
	StddevMulThresh = GetProgramOptionAsDouble(argc, argv, "--StddevMulThresh", 2);
	save_format = GetProgramOptionAsInt(argc, argv, "--format", 1);

	if(path_source.find(".pcd")==-1 && path_source.find(".ply")==-1){
		PCL_ERROR("input --path_source  with extension .pcd or .ply\n");
		return 0;
	}
	if(path_target.find(".pcd")==-1 && path_target.find(".ply")==-1){
		PCL_ERROR("input --path_target  with extension .pcd or .ply\n");
		return 0;
	}

	std::cout << "Use parameters:\n"
		<< "source_pcd_path: " << path_source << std::endl
		<< "target_pcd_path: " << path_target << std::endl
		<< "MeanK: " << MeanK << std::endl
		<< "StddevMulThresh: " << StddevMulThresh << std::endl
		<< "format to save: " << ((save_format == 0) ? "ascii" : "binary") << std::endl;
	clock_t read_time_s = clock();
	if (!pointcloudread_pcd(path_source, pcd_source)) {
		return 0;
	}
	clock_t read_time = clock();
	std::cout << "read pcd cost: " << (double)(read_time - read_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	//filter
	clock_t filter_time_s = clock();
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(pcd_source);
	sor.setMeanK(MeanK);
	sor.setStddevMulThresh(StddevMulThresh);
	sor.filter(*pcd_filtered);
	
	clock_t filter_time = clock();
	std::cout << "filter pcd cost: " << (double)(filter_time - filter_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	clock_t save_pcd_time_s = clock();

	if(path_target.find(".pcd")!=-1){
	pcl::PCDWriter writer;
	std::cout<<"the filtered pcd has saved to "<<path_target<<std::endl;
	writer.write<PointT>(path_target,*pcd_filtered,save_format);
	}
	if(path_target.find(".ply")!=-1){
	pcl::PLYWriter writer;
	std::cout<<"the filtered pcd has saved to "<<path_target<<std::endl;
	writer.write<PointT>(path_target,*pcd_filtered,save_format);
	}
	clock_t save_pcd_time = clock();
	std::cout << "save pcd cost: " << (double)(save_pcd_time - save_pcd_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	return 0;


}