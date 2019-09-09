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
    cv::Scalar minRGB = cv::Scalar(-1, -1, -1);
    cv::Scalar maxRGB = cv::Scalar(-1, -1, -1);
    cv::Scalar minHSV = cv::Scalar(-1, -1, -1);
    cv::Scalar maxHSV = cv::Scalar(-1, -1, -1);
};

#endif