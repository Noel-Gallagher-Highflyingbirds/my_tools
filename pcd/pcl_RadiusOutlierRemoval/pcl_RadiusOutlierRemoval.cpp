#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/time.h>   // TicToc 
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>

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
	std::cout << "after remove NAN£¬remain " << cloud->points.size() << std::endl;

	clock_t read_time = clock();
	std::cout << "\nLoaded file " << filename << " (" << cloud->size() << " points) in "
		<< (double)(read_time - read_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	return true;
}

int main(int argc,char** argv) {
	PointCloudT::Ptr pcd_source(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr pcd_filtered(new PointCloudT);  // Original point cloud
	std::string source_pcd_path;
	std::string target_pcd_path = "./pcd_RadiusOutlierRemoval.pcd";
	float radius_search = 10.0;
	int min_neighbors_in_radius = 20;

	if (argc < 3 || argv[1] == "-h" || argv[1] == "--h") {
		std::cout << "input 4 parameters:\n"
			<< "source point cloud path.\n"
			<< "point cloud path to save downsample pcd. defalut: " << target_pcd_path << std::endl
			<< "RadiusSearch(float): Radius to search points: " << radius_search << std::endl
			<< "MinNeighborsInRadius(int), default: " << min_neighbors_in_radius << std::endl;

	}
	source_pcd_path = argv[1];
	if (argc >= 3) {
		target_pcd_path = argv[2];
	}
	if (argc >= 4) {
		radius_search = atof(std::string(argv[3]).c_str());
	}
	if (argc >= 5) {
		min_neighbors_in_radius = atoi(std::string(argv[4]).c_str());
	}

	std::cout << "Use parameters:\n"
		<< "source_pcd_path: " << source_pcd_path << std::endl
		<< "target_pcd_path: " << target_pcd_path << std::endl
		<< "RadiusSearch: " << radius_search << std::endl
		<< "MinNeighborsInRadius: " << min_neighbors_in_radius << std::endl;
	clock_t read_time_s = clock();
	if (!pointcloudread_pcd(source_pcd_path, pcd_source)) {
		return 0;
	}
	clock_t read_time = clock();
	std::cout << "read pcd cost: " << (double)(read_time - read_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	//downsample
	clock_t downsample_time_s = clock();
	pcl::RadiusOutlierRemoval<PointT> outrem;
	// build the filter
	outrem.setInputCloud(pcd_source);
	outrem.setRadiusSearch(radius_search);
	outrem.setMinNeighborsInRadius(min_neighbors_in_radius);
	outrem.setKeepOrganized(true);
	// apply filter
	outrem.filter(*pcd_filtered);
	
	clock_t downsample_time = clock();
	std::cout << "filter pcd cost: " << (double)(downsample_time - downsample_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	clock_t save_pcd_time_s = clock();
	pcl::io::savePCDFileBinaryCompressed(target_pcd_path, *pcd_filtered);
	clock_t save_pcd_time = clock();
	std::cout << "save pcd cost: " << (double)(save_pcd_time - save_pcd_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	return 0;


}