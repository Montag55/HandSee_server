#include <stdio.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {

    std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
    std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;
    std::cout << "Build info : " << cv::getBuildInformation().c_str() << std::endl;

    if (argc != 2)
    {
        printf("usage: $./Quality <Image_Path>\n");
        return -1;
    }


    cv::Mat image = cv::imread(argv[1], 1);
    if (!image.data) {
        printf("No image data \n");
        return -1;
    } 
    else {
        cv::imwrite("test.jpg", image);
    }
    
    return 0;
}