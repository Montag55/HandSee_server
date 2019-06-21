#include "server.hpp"
#include <string>
#include <iostream>
#include <stdio.h>

Server::Server(char* server_ip, char* server_port, int buffer_size):
  m_server_ip {server_ip},
  m_server_in_port {std::stol(server_port)},
  m_buffer_size {buffer_size}
  {
    m_buffer.resize(m_buffer_size);
  }

Server::~Server(){}

bool Server::run(){
  int server_fd, new_socket, valread;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);

  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
      std::cout << "socket failed" << std::endl;
      exit(EXIT_FAILURE);
  }

  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
      std::cout << "setsockopt" << std::endl;
      exit(EXIT_FAILURE);
  }

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(m_server_in_port);

  if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
      std::cout << "bind failed" << std::endl;
      exit(EXIT_FAILURE);
  }

  if (listen(server_fd, 3) < 0) {
      std::cout << "listen" << std::endl;
      exit(EXIT_FAILURE);
  }

  if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
      std::cout << "accept" << std::endl;
      exit(EXIT_FAILURE);
  }
  
  // while(valread = read( new_socket , m_buffer.data(), m_buffer.size()) > 0 ){
  //   for(unsigned int i = 0; i < m_buffer.size(); i++){
  //     if(&m_buffer[i] != NULL)
  //       std::cout << "i: " << i << "; " << &m_buffer[i] << std::endl;
  //   }
  // }

  int bytesReceived = 0;
  FILE *image;
  char buffer[1024] = {0};
  image = fopen("recu.jpg", "wba");

  while (bytesReceived < sizeof(buffer)){
    std::cout << "Reading image byte array" << std::endl;
    int n = 0;
    if ((n = recv(new_socket, buffer, sizeof(buffer), 0)) < 0)
    {
      perror("recv_size()");
      exit(errno);
    }
    bytesReceived += n;
    std::cout << n << std::endl;
    std::cout << "Converting byte array to image" << std::endl;
    fwrite(buffer, sizeof(char), n, image);
    std::cout << "done" << std::endl;

    printf("%s\n", buffer);
    char *response = "Message Received!";
    send(new_socket, response, strlen(response), 0);
  }
  fclose(image);

  return EXIT_FAILURE;
}

cv::Mat Server::creatDisplacementMap(){
  // Note: currently reading from file
  cv::Mat testIMG = cv::imread("./test.jpg", 1);
  if (!testIMG.data) {
    std::cout << "\nError: no source image data.\n" << std::endl;
  }

  cv::cvtColor(testIMG, testIMG, 6);
  std::tuple<cv::Mat, cv::Mat> splitImages = splitImage(testIMG);
  
  
  // Downscales the images by 0.5
  // cv::Size downScale(std::get<0>(splitImages).size().width / 2, std::get<0>(splitImages).size().height / 2);
  // cv::resize(std::get<0>(splitImages), std::get<0>(splitImages), downScale);
  // cv::resize(std::get<1>(splitImages), std::get<1>(splitImages), downScale);
  // std::cout << std::get<0>(splitImages).size().width << "x" << std::get<0>(splitImages).size().height << std::endl;
  // std::cout << std::get<1>(splitImages).size().width << "x" << std::get<1>(splitImages).size().height << std::endl;
  
  
  // init mats used for calulation
  cv::Mat disp_left, disp_right, filteredDisp, filter_Disp_vis;
  cv::Mat left = std::get<0>(splitImages);
  cv::Mat right = std::get<1>(splitImages);
  

  // computes disparity map left->right & right->left
  matcher_left->compute(std::get<0>(splitImages), std::get<1>(splitImages), disp_left);
  matcher_right->compute(std::get<1>(splitImages), std::get<0>(splitImages), disp_right);


  // create matchers for filtering methode
  cv::Ptr<cv::StereoSGBM> matcher_left = cv::StereoSGBM::create(0, 16, 5, 0, 0, 0, 0, 0, 0, 0, cv::StereoSGBM::MODE_HH );
  cv::Ptr<cv::StereoSGBM> matcher_right = cv::StereoSGBM::create(0, 16, 5, 0, 0, 0, 0, 0, 0, 0, cv::StereoSGBM::MODE_HH );
  // cv::Ptr<cv::StereoMatcher> matcher_right = cv::ximgproc::createRightMatcher(matcher_left);
  

  // apply filters on computed disparity maps
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher_left);
  wls_filter->setLambda(8000.0);
  wls_filter->setSigmaColor(1.5);
  wls_filter->filter(disp_left, left, filteredDisp, disp_right);
  cv::ximgproc::getDisparityVis(filteredDisp, filter_Disp_vis, 2);


  // normalize image values and save
  cv::normalize(filter_Disp_vis, filter_Disp_vis, 255, 0, cv::NORM_MINMAX);
  saveImg(filter_Disp_vis, "disparityMapfiltered.jpg");
  return testIMG;
}

std::tuple<cv::Mat, cv::Mat> Server::splitImage(cv::Mat inputIMG){
  /*
   * Note: Currently not entire image is used. 13 pixel rows are left out since
   * the mirror edge crosses all of these (as far as i can tell by using GIMP)
  **/
  cv::Rect upperRect(0, 0, inputIMG.size().width, 617);
  cv::Rect lowerRect(0, 630, inputIMG.size().width, inputIMG.size().height - (630 + 33));
  
  cv::Mat upperReflection = inputIMG(upperRect);
  cv::Mat lowerReflection = inputIMG(lowerRect);
  cv::flip(lowerReflection, lowerReflection, 0);
  saveImg(lowerReflection, "flip.jpg");

  transpose(lowerReflection, lowerReflection);
  transpose(upperReflection, upperReflection);

  cv::flip(lowerReflection, lowerReflection, 1);
  cv::flip(upperReflection, upperReflection, 1);
  saveImg(lowerReflection, "flip1.jpg");
  saveImg(upperReflection, "flip2.jpg");

  std::tuple<cv::Mat, cv::Mat> images(lowerReflection, upperReflection);

  return images;
}

void Server::saveImg(cv::Mat img, std::string filename){
  if (cv::imwrite(filename, img)){
    std::cout << "Saved image to: " << "build/" << filename << std::endl;
  }
  else {
    std::cout << "Error: could not save image to: " << "build/" << filename << "." << std::endl;
  }
}
