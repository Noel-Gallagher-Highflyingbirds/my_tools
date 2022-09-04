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
    using namespace open3d;

    PrintOpen3DVersion();
   
    std::cout << "Input parameters:" << std::endl
        << "--path_source: path to a pcd need to be register" << std::endl
        << "--path_target: path to the target pcd" << std::endl
        << "--path_matrix file: path to the matrix file, will apply to the source_pcd" << std::endl
        << "\nnote: the matrix in txt must be like:\n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n" << std::endl;

}

int main(int argc, char* argv[]) {
    using namespace open3d;

    clock_t start_time = clock();

    std::string path_source="";
    std::string path_target="";
    std::string path_matrix="";
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 4 ||
        utility::ProgramOptionExistsAny(argc, argv, { "-h", "--help" })) {
        PrintHelp();
        return 1;
    }
    

    std::shared_ptr<geometry::PointCloud> source(new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target(new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> source_trans(new geometry::PointCloud);

    path_source = utility::GetProgramOptionAsString(argc, argv, "--path_source", "");
    if (path_source == "") {
        std::cout << "please check your source pcd path: " << path_source << std::endl;
        return 0;
    }
    path_target = utility::GetProgramOptionAsString(argc, argv, "--path_target", "");
    if (path_target == "") {
        std::cout << "please check your target pcd path: " << path_target << std::endl;
        return 0;
    }

    path_matrix = utility::GetProgramOptionAsString(argc, argv, "--path_matrix", "");


    source = open3d::io::CreatePointCloudFromFile(path_source);
    source->PaintUniformColor({ 0,1,0 });

    if (argc >= 3) {
        target = open3d::io::CreatePointCloudFromFile(path_target);
        target->PaintUniformColor({ 1,0,0 });
    }

    visualization::DrawGeometries(
        { source, target },
        "visualize", 960, 900, 960, 100);
    

    
    if (path_matrix != "") {
        Eigen::Matrix4d_u trans_matrix = Eigen::Matrix4d_u::Identity();

        std::ifstream fin(path_matrix, std::ios::in);
        if (!fin.is_open()) {
            std::cout << "failed to open: " << path_matrix << "please check it\n";
            return 0;
        }

        float value = 0;
        for (std::size_t i = 0; i < 4; ++i) {
            for (std::size_t j = 0; j < 4; ++j) {
                fin >> value;
                trans_matrix(i, j) = value;
            }
        }
        fin.close();
        std::cout << std::endl
            << "trans_matrix:" << std::endl
            << trans_matrix << std::endl;

        *source_trans = *source;

        source_trans->PaintUniformColor({ 0,0,1 });
        source_trans->Transform(trans_matrix);

        visualization::DrawGeometries(
            { target, source_trans },
            "visualize", 960, 900, 960, 100);
        visualization::DrawGeometries(
            { source, target,source_trans },
            "visualize", 960, 900, 960, 100);

    }
  

    return 0;
}
