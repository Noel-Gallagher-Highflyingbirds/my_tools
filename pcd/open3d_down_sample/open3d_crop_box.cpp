#include <iostream>
#include <memory>
#include <thread>
#include <Eigen/Dense>
#include <open3d/Open3D.h>
using namespace open3d;

int main(int argc, char** argv)
{

    std::string ply_filename;
    if (argc < 2) {
        std::cout << "please input a pcd file" << std::endl;
        return 0;
    }
    ply_filename = argv[1];
    std::cout << ply_filename << std::endl;
    auto source_ptr = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(ply_filename, *source_ptr)) {
        utility::LogInfo("Successfully read {}\n", ply_filename);
    }
    else {
        utility::LogWarning("Failed to read {}\n\n", ply_filename);
        return 1;
    }
    visualization::DrawGeometriesWithEditing({ source_ptr }, "editname", 1920, 1080);

    return 0;
}