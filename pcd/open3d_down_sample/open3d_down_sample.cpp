// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "open3d/Open3D.h"

using namespace open3d;

void PrintHelp() {

    std::cout << "Input parameters:" << std::endl
        << "--path_source path: path to a las pcd need to be downsample" << std::endl
        << "--path_target path: path to save the downsampled pcd" << std::endl
        << "--voxel_size: voxel size to downsample pcd,defalut 2.0" << std::endl
        << "--visualize: to visualize the pcd" << std::endl;


}


int main(int argc, char* argv[]) {

    /*
    * By carlos_lee 20220306
    * pcd downsample
    */
    std::string path_source;
    std::string path_target;
    double voxel_size = 2.0;

    clock_t start_time = clock();
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 5 || utility::ProgramOptionExistsAny(argc, argv, { "-h", "--help" })) {
        PrintHelp();
        return 0;
    }
    path_source = utility::GetProgramOptionAsString(argc, argv, "--path_source", "");
    path_target = utility::GetProgramOptionAsString(argc, argv, "--path_target", "");
    voxel_size = utility::GetProgramOptionAsDouble(argc, argv, "--voxel_size", 2.0);
    bool visualize = false;
    if (utility::ProgramOptionExists(argc, argv, "--visualize")) {
        visualize = true;
    }
    std::cout << voxel_size << std::endl;

    //read pcd from file
    auto pcd = open3d::io::CreatePointCloudFromFile(path_source);
    clock_t read_time = clock();
    std::cout << "read pcd cost: " << (double)(read_time - start_time) / (double)CLOCKS_PER_SEC << " s" << std::endl;

    //downsample
    clock_t downsample_time_s = clock();
    auto pcd_down = pcd->VoxelDownSample(voxel_size);
    clock_t downsample_time = clock();
    std::cout << "downsample time: " << (double)(downsample_time - downsample_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;
    //save pcd
    open3d::io::WritePointCloud(path_target, *pcd_down);
    clock_t write_time = clock();
    std::cout << "write file time: " << (double)(write_time - downsample_time) / (double)CLOCKS_PER_SEC << " s" << std::endl;

    if (visualize) {
        visualization::DrawGeometries(
            { pcd },
            "origin pcd", 960, 900, 960, 100);
        visualization::DrawGeometries(
            { pcd_down },
            "downsampled pcd", 960, 900, 960, 100);
    }

    return 0;
}
