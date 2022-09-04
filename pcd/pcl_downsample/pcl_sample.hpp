#include "pcl_sample.h"
/*
* author:carlos_lee 20220306
* functions used for pcd sampling, implementation with pcl
*/
void down_sample_voxel(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_filtered, float grid_size, bool origin_point) {
	/*
	* voxelgrid downsample generate a new set of points, which every point is the center of a voxel,
	* so downsample point cloud are different from the origin one.
	* if origin_point=true, every point p_v generate from VoxelGrid downsample
	* will search its nearest neighbor p_o in the origin point cloud,
	* the set of p_o will be the final output---P_o.
	* so every downsample point p_o are select from the origin data,
	* and due to the voxel grid, P_o will be more uniform than ordinary random sampling points
	*/
	/*源码里面有一个体素内点数大于阈值才予以计算,默认值为0*/
	std::cout << "Begin to VoxelGrid sample.\nDownsampling...\n";
	clock_t voxel_s = clock();
	pcl::VoxelGrid<PointT> voxel_grid1;
	voxel_grid1.setLeafSize(grid_size, grid_size, grid_size);
	voxel_grid1.setInputCloud(cloud_in);
	voxel_grid1.filter(*cloud_filtered);
	std::cout << "the origin pcd has " << cloud_filtered->size() << "points\n"
		<< "downsampled with voxel grid = " << grid_size << ", left " << cloud_filtered->size() << "points" << std::endl;
	clock_t voxel_e = clock();
	std::cout << std::endl << "VoxelGrid downsample time:" << (double)(voxel_e - voxel_s) << " ms";//时间显示

	std::cout << "KNN searching...\n";
	if (origin_point) {
		pcl::KdTreeFLANN<PointT>kdtree;
		kdtree.setInputCloud(cloud_in);
		clock_t search_s = clock();
		std::vector<int> index;
		std::vector<int> indextemp(1);
		std::vector<float> distancetemp(1);
		std::vector<float> distance;
		for (size_t idx = 0; idx < cloud_filtered->points.size(); ++idx)
		{
			//对体素降采样后的点，在原点云中进行最近邻查找
			kdtree.nearestKSearch(cloud_filtered->points[idx], 1, indextemp, distancetemp);
			index.push_back(indextemp[0]);
			distance.push_back(distancetemp[0]);
		}
		clock_t search_e = clock();
		std::cout << std::endl << "kdtree searchtime:" << (double)(search_e - search_s) << " ms";

		clock_t rechange_s = clock();
		for (size_t idx2 = 0; idx2 < cloud_filtered->points.size(); ++idx2)
		{
			//最近点替换
			cloud_filtered->points[idx2] = cloud_in->points[index[idx2]];
		}
		clock_t rechange_e = clock();
		std::cout << std::endl << "replace voxel center with origin pionts cost:" << (double)(rechange_e - rechange_s) << " ms";//时间显示
	}

}

void down_sample_random(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_filtered, int random_subsample)
{
	std::cout << "Begin to ordinary random sample.\n";
	clock_t start_time = clock();
	//if (random == 1)
	//	srand((unsigned int)time(0));
	std::random_shuffle(cloud_in->begin(), cloud_in->end());
	//*cloud_icp_toberegistration = *cloud_toberegistration;
	if (cloud_in->points.size() < random_subsample) {
		random_subsample = cloud_in->points.size();
	}
	if (&cloud_filtered != &cloud_in) {
		*cloud_filtered = *cloud_in;
	}
	cloud_filtered->points.resize(random_subsample);//default 2000
	cloud_filtered->width = random_subsample;
	std::cout << "cloudlas->points.size() after sample: " << cloud_filtered->points.size() << std::endl;
	clock_t random_subsample_time = clock();
	std::cout << std::endl << "random sample cost: " << (double)(random_subsample_time - start_time) << " ms\n";//时间显示

}

void down_sample_uniform(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_filtered, float grid_size) {
	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud(cloud_in);
	uniform_sampling.setRadiusSearch(grid_size);
	uniform_sampling.filter(*cloud_filtered);
}

