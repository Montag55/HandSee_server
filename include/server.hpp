#ifndef SERVER_HPP
#define SERVER_HPP

#include <vector>
#include <unistd.h>
#include <memory>
#include <tuple>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "imgFeatures.hpp"

class Server{
  public:
    Server(char* server_ip, char* server_port, int buffer_size);
    ~Server();
    
    bool run();
    cv::Mat createDisplacementMap();
    void createSkinMask();
    void splitImage();
    void saveImg(cv::Mat img, std::string filename);
    void sendMessage(int connfd, char *format);
    void readVideoImgFromDisk(std::string filePath = "./../Test/frame%01d.jpg", int mode = cv::CAP_IMAGES, int length = 1, int start = 1);
    void initializeMatchers(int window_size = 5, int P1 = 24, int P2 = 96, int UniqRatio = 0, int maxDiff = 80000, int specklSize = 0, int filterCap = 63, int mode = cv::StereoSGBM::MODE_HH);
    void initializeWLSFilter(int window_size = 5, double lambda = 8000.0, double sigmeColor = 1.2, int depthRadius = (int) ceil(0 * 5));
    void initializeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance);
    void updateImageFeatures();
    void rectification();
    std::vector<cv::Scalar> retrieveAvgDisplacement(cv::Mat disMap);
    void calibrateColor();
    void getContours();
    int findHand(std::vector<cv::Point> contour, std::vector<int> hull, std::vector<cv::Vec4i> hullDefects, cv::Mat tmp);
    float calcAngle(cv::Point f, cv::Point s, cv::Point e);

  private : 
    char *m_server_ip;
    long m_server_in_port;
    int m_buffer_size;
    int m_leftRightSwitch;

    cv::Mat m_left, m_right, m_skinMask, m_colHist, m_srcImg;

    bool m_initializationStatus;
    std::vector<char> m_buffer;
    std::vector<cv::Point> m_fingers;
    cv::Ptr<cv::StereoSGBM> m_leftMatcher;
    cv::Ptr<cv::StereoMatcher> m_rightMatcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> m_wlsFilter;
    cv::Rect m_ROI;
    cv::Point m_centroid;
    cv::VideoCapture m_video;
    ImgFeatures m_imgFeatures;

};

#endif
