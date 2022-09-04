#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#define PI 3.1415926

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "please input a file including a 3x3 matrix" << std::endl;
        return 0;
    }
    std::string filename;
    filename = argv[1];
    std::ifstream fin;
    fin.open(filename, std::ios::in);
    if (!fin.is_open()) {
        std::cout << "read file " << filename << " failed." << std::endl;
        return 0;
    }
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            double value;
            fin >> value;
            // std::cout<<value<<" ";
            rotation_matrix(i, j) = value;
        }
    }
    std::cout << std::endl;
    fin.close();

    std::cout << "rotation_matrix:\n" << rotation_matrix << std::endl;
    Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(0, 1, 2);
    // std::cout<<eulerAngle<<std::endl;
    std::cout << "eulerAngle:\nx: " << eulerAngle[0] / PI * 180 << "    y: " << eulerAngle[1] / PI * 180 << "    z: " << eulerAngle[2] / PI * 180 << std::endl;

    return 0;
}
