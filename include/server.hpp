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
    cv::Mat creatDisplacementMap();
    std::tuple<cv::Mat, cv::Mat> splitImage(cv::Mat inputIMG);
    void saveImg(cv::Mat img, std::string filename);

  private : 
    char *m_server_ip;
    long m_server_in_port;
    int m_buffer_size;
    std::vector<char> m_buffer;
};

#endif
