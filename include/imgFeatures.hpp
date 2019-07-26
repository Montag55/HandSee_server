#ifndef IMGFEATURES_HPP
#define IMGFEATURES_HPP

#include <opencv2/opencv.hpp>
#include "opencv2/ximgproc.hpp"

struct ImgFeatures {

    float imgScale = 1.0f;
    float rotAngleL2R = 0.0f;
    std::vector<cv::KeyPoint> keyPoints;
    cv::Point2i leftMirrorPoint;
    cv::Point2i rightMirrorPoint;
    cv::Ptr<cv::xfeatures2d::SIFT> sift;
};


#endif