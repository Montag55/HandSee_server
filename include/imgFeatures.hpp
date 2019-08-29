#ifndef IMGFEATURES_HPP
#define IMGFEATURES_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

struct ImgFeatures {
    float imgScale = 1.0f;
    cv::Mat m_H1;
    cv::Mat m_H2;
    cv::Mat m_F;
};


#endif