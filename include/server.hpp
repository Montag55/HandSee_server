#ifndef SERVER_HPP
#define SERVER_HPP

#include <vector>
#include <unistd.h>
#include <memory>
#include <tuple>
#include <sys/socket.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include "opencv2/ximgproc.hpp"

class Server{
  public:
    Server(char* server_ip, char* server_port, int buffer_size);
    ~Server();
    
    bool run();
    cv::Mat creatDisplacementMap(cv::Mat inputImg = cv::imread("./../test3.jpg", 1));
    cv::Mat creatSkinMask(const cv::Mat& srcImg);
    std::tuple<cv::Mat, cv::Mat> splitImage(const cv::Mat& inputIMG, float scale = 1.0f);
    void saveImg(cv::Mat img, std::string filename);
    void sendMessage(int connfd, char *format);
    void readVideoImgFromDisk(int length = 30, int start = 0);
    void initializeMatchers(int window_size = 5, int P1 = 24, int P2 = 96, int UniqRatio = 0, int maxDiff = 80000, int specklSize = 0, int filterCap = 63, int mode = cv::StereoSGBM::MODE_HH);
    void initializeWLSFilter(int window_size = 5, double lambda = 8000.0, double sigmeColor = 1.2, int depthRadius = (int) ceil(0 * 5));
    void initializeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance);

  private:
    char *m_server_ip;
    long m_server_in_port;
    int m_buffer_size;
    bool m_initializationStatus;
    std::vector<char> m_buffer;
    cv::Ptr<cv::StereoSGBM> m_leftMatcher;
    cv::Ptr<cv::StereoMatcher> m_rightMatcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> m_wlsFilter;
    cv::Rect m_ROI;
};

#endif
