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
#include <fstream>
#include "open3d/Open3D.h"

using namespace open3d;
using namespace std;





void PrintHelp() {

    PrintOpen3DVersion();
    std::cout << "Input parameters:" << std::endl
        << "path_source: path to a pcd need to be visulize" << std::endl;
}

int main(int argc, char* argv[]) {

    std::string path_source = "";
    clock_t start_time = clock();

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 2 ||
        utility::ProgramOptionExistsAny(argc, argv, { "-h", "--help" })) {
        PrintHelp();
        return 1;
    }
    path_source = utility::GetProgramOptionAsString(argc, argv, "--path_source", "");
    if (path_source == "") {
        std::cout << "please check your source pcd path: " << path_source << std::endl;
        return 0;
    }

    std::shared_ptr<geometry::PointCloud> source(new geometry::PointCloud);
    clock_t read_time_s = clock();
    open3d::io::ReadPointCloud(path_source, *source);
    clock_t read_time = clock();
    std::cout << "\nLoaded file " << path_source << " (" << source->points_.size() << " points) in "
        << (double)(read_time - read_time_s) / (double)CLOCKS_PER_SEC << " s" << std::endl;

    std::cout << "read " << source->points_.size() << "from file" << std::endl;
    source->RemoveNonFinitePoints();
    std::cout << "After remove NAN, " << source->points_.size() << " points remain" << std::endl;

    visualization::DrawGeometries(
    { source},
    "visualize with origin color", 960, 900, 960, 100);

    source->PaintUniformColor({1,0,0});
    visualization::DrawGeometries(
        { source },
        "visualize with red color", 960, 900, 960, 100);

    
    return 0;
}
