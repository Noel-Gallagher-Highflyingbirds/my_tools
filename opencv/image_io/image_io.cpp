#include <iostream>

#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc_c.h>
// #include <opencv2/core.hpp>
// #include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char* argv[]){
    if(argc<2){
        std::cout<<"please input a image path"<<std::endl;
        return 0;
    }
    std::string filename=argv[1];
    cv::Mat img;
    img = cv::imread(filename);
    cv::namedWindow("show",cv::WINDOW_NORMAL);
    cv::resizeWindow("show",640,480);
    cv::imshow("show",img);
    cv::waitKey(0);
    cv::imwrite("./img_write.jpg",img);
    return 0;

}