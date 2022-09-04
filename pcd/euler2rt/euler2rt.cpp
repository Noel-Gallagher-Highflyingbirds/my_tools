#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#define PI 3.1415926

int main(int argc, char* argv[]){
    std::cout<<PI<<std::endl;
    if(argc<4){
        std::cout<<"please input a 3x1 vector,for example:\neuler2rt 45 30 60"<<std::endl;
        return 0;  
    }
    
    Eigen::Vector3d eulerAngle(atof(argv[1]),atof(argv[2]),atof(argv[3]));
    std::cout<<"eulerAngle:\nx: "<<eulerAngle[0]<<"    y: "<<eulerAngle[1]<<"    z: "<<eulerAngle[2]<<std::endl;
    eulerAngle=eulerAngle/180*PI;


    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle[0],Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle[1],Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle[2],Eigen::Vector3d::UnitZ()));


    rotation_matrix=rollAngle*pitchAngle*yawAngle;

    std::cout<<"rotation_matrix:\n"<<rotation_matrix<<std::endl;
    Eigen::Vector3d eulerAngle2=rotation_matrix.eulerAngles(0,1,2);
     std::cout<<"eulerAngle:\nx: "<<eulerAngle2[0]/PI*180<<"    y: "<<eulerAngle2[1]/PI*180<<"    z: "<<eulerAngle2[2]/PI*180<<std::endl;

    return 0;
}