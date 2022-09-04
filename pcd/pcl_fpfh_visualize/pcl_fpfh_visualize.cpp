#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h>
//#include "pcl_sample.hpp"
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


// Mutex: //
boost::mutex cloud_mutex;
pcl::visualization::PCLPlotter plotter;
pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

struct callback_args {
	// structure used to pass arguments to the callback function
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	plotter.clearPlots();
	struct callback_args* data = (struct callback_args*)args;
	if (event.getPointIndex() == -1)
		return;
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.clear();
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in green:
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 0, 255, 0);
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

	int num = event.getPointIndex();
	plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh", num);
	plotter.plot();
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
	* carlos_lee 20200319
	*/
	std::string pcd_path;
	float voxel_size = 2.0;
	int fix_points = 0;
	if (argc < 3 || argv[1] == "-h" || argv[1] == "--h") {
		std::cout << "input parameters:\n"
			<< "point cloud path:xxx.pcd\n"
			<< "voxelsize, for downsample，default: " << voxel_size << std::endl;
		return 0;

	}

	if (argc >= 2) {
		pcd_path = argv[1];
	}

	if (argc >= 3) {
		voxel_size = atof(std::string(argv[2]).c_str());
	}
	std::cout << "parameters:\n"
		<< "point cloud path: " << argv[1] << std::endl
		<< "voxel_size: " << voxel_size << std::endl;


	//visualizer
	PointCloudT::Ptr cloud(new PointCloudT);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

	if (!pointcloudread_pcd(pcd_path, cloud)) {
		return 0;
	}

	cloud_mutex.lock();// for not overwriting the point cloud
	PointCloudT::Ptr cloud_filltered(new PointCloudT);
	*cloud_filltered = *cloud;
	if (voxel_size != 0) {
		PointCloudT::Ptr cloud_filltered(new PointCloudT);
		pcl::VoxelGrid<PointT> grid;
		grid.setInputCloud(cloud);
		grid.setLeafSize(voxel_size, voxel_size, voxel_size);
		grid.filter(*cloud_filltered);
	}
	clock_t normal_time_s = clock();
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
	ne.setInputCloud(cloud_filltered);
	ne.setSearchSurface(cloud);
	ne.setNumberOfThreads(4);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(voxel_size*2);
	ne.compute(*cloud_normals);
	clock_t normal_time = clock();
	std::cout << "\nNormalEstimationOMP cost: " << (double)(normal_time - normal_time_s) / (double)CLOCKS_PER_SEC << " s\n" << std::endl;

	clock_t fpfh_time_s = clock();
	fpfh.setInputCloud(cloud_filltered);
	fpfh.setInputNormals(cloud_normals);
	pcl::search::KdTree<PointT>::Ptr tree_1(new pcl::search::KdTree<PointT>);
	fpfh.setSearchMethod(tree_1);
	fpfh.setRadiusSearch(voxel_size * 5);
	fpfh.compute(*fpfhs);
	clock_t fpfh_time = clock();
	std::cout << "\nNormalEstimationOMP cost: " << (double)(fpfh_time - fpfh_time_s) / (double)CLOCKS_PER_SEC << " s\n" << std::endl;

	viewer->setBackgroundColor(255, 255, 255);
	// Display pointcloud:
	viewer->addPointCloud(cloud_filltered, "cloud_filltered");
	//viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

	// Add point picking callback to viewer:
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

	// Spin until 'Q' is pressed:
	viewer->spin();
	std::cout << "done." << std::endl;

	cloud_mutex.unlock();

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}

	system("pause");
	return 0;












	//std::string source_pcd_path;
	//std::string target_pcd_path;
	//int save_format = 0;
	//int fix_points = 0;
	//if (argc < 3 || argv[1] == "-h" || argv[1] == "--h") {
	//	std::cout << "input parameters:\n"
	//		<< "source point cloud path:xxx.ply\n"
	//		<< "point cloud path to save pcd.\n"
	//		<< "format to save: binary(0), ascii(1), default: " << save_format << std::endl
	//		<< "fix_points: points you want remain after downsample,default 0, it means no downsampling.\n";
	//	return 0;

	//}
	//
	//if (argc >= 2) {
	//	source_pcd_path = argv[1];
	//}	
	//if (argc >= 3) {
	//	target_pcd_path = argv[2];
	//}
	//if (argc >= 4) {
	//	save_format = atoi(std::string(argv[3]).c_str());
	//}
	//if (argc >= 5) {
	//	fix_points = atoi(std::string(argv[4]).c_str());
	//}
	//std::cout << "parameters:\n"
	//	<< "source point cloud path: " << argv[1] << std::endl
	//	<< "point cloud path to save pcd: " << target_pcd_path << std::endl
	//	<< "format to save: " << ((save_format == 0) ? "binary" : "ascii") << std::endl
	//	<< "fix_points: " << fix_points << std::endl;


	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	//if (!pointcloudread_pcd(source_pcd_path, cloud)){
	//	return -1;
	//}

	//if (fix_points != 0) {
	//	std::cout << "downsampling...\n";
	//	down_sample_random(cloud, cloud, fix_points);
	//	
	//}
	//
	//clock_t write_time_s = clock();
	//
	//if (save_format == 0) {
	//	pcl::io::savePLYFileBinary(target_pcd_path, *cloud);//存储为pcd类型文件
	//}
	//else {
	//	pcl::io::savePLYFileASCII(target_pcd_path, *cloud);//存储为pcd类型文件
	//}
	//clock_t write_time = clock();
	//std::cout << "write pcd cost: " << (double)(write_time - write_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	//return (0);
}