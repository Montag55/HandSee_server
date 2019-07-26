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
    m_imgFeatures.sift = cv::xfeatures2d::SIFT::create();
    m_initializationStatus = false;

    initializeMatchers();
    initializeWLSFilter();
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
    if ((n = recv(new_socket, buffer, sizeof(buffer), 0)) < 0) {
      perror("recv_size()");
      exit(errno);
    }
    
    bytesReceived += n;
    std::cout << n << std::endl;
    std::cout << "Converting byte array to image" << std::endl;
    fwrite(buffer, sizeof(char), n, image);
    std::cout << "done" << std::endl;

    printf("%s\n", buffer);
    sendMessage(new_socket, "Message Received!");
  }
  fclose(image);

  return EXIT_FAILURE;
}

cv::Mat Server::creatDisplacementMap(cv::Mat inputImg){

  /* vars for use */
  cv::Mat disp_left, disp_right, filteredDisp, raw_Disp_vis, filter_Disp_vis;
  
  /* Create left and right images & create Skinmask for left image */
  cv::Mat skinMask = creatSkinMask(std::get<0>(splitImage(inputImg)));
  cv::cvtColor(inputImg, inputImg, cv::COLOR_BGR2GRAY);
  std::tuple<cv::Mat, cv::Mat> splitImages = splitImage(inputImg);
  cv::Mat left = std::get<0>(splitImages);
  cv::Mat right = std::get<1>(splitImages);

  /* initiaize RIO if not yet done */
  if (m_initializationStatus == false){
    initializeROI(left.size(), m_leftMatcher);
    m_initializationStatus = true;
  }
  
  /* compute left and right disparity matcher */
  m_leftMatcher->compute(left, right, disp_left);
  m_rightMatcher->compute(right, left, disp_right);
  disp_left.convertTo(disp_left, CV_16S);
  disp_right.convertTo(disp_right, CV_16S);

  /* compute disparity map */
  m_wlsFilter->filter(disp_left, std::get<0>(splitImages), filteredDisp, disp_right, m_ROI, std::get<1>(splitImages));
  cv::ximgproc::getDisparityVis(filteredDisp, filter_Disp_vis, 2.0f);

  /* apply skin mask on disparity map*/
  cv::Mat masked;
  cv::normalize(filter_Disp_vis, filter_Disp_vis, 255.0f, 0, cv::NORM_MINMAX);
  cv::bitwise_and(filter_Disp_vis, filter_Disp_vis, masked, skinMask);

  saveImg(masked, "imgDisp.jpg");
  return masked;
}

std::tuple<cv::Mat, cv::Mat> Server::splitImage(const cv::Mat& inputIMG){
  /*
   * Note: Currently not entire image is used. 13 pixel rows are left out since
   * the mirror edge crosses all of these (as far as i can tell by using GIMP)
  **/

  /* Clone original image, otherwise overwrite */
  cv::Mat srcImg = inputIMG.clone();

  /* Scale image */
  cv::Size downScale(srcImg.size().width * m_imgFeatures.imgScale, srcImg.size().height * m_imgFeatures.imgScale);
  cv::resize(srcImg, srcImg, downScale);

  /* Create upper and lower image regions */
  cv::Rect upperRect(0, 0, srcImg.size().width, 617 * m_imgFeatures.imgScale);
  cv::Rect lowerRect(0, 630 * m_imgFeatures.imgScale, srcImg.size().width, srcImg.size().height - (630 * m_imgFeatures.imgScale + 33 * m_imgFeatures.imgScale));

  /* Rotate/mirror images so the have same orientation */
  cv::Mat upperReflection = srcImg(upperRect);
  cv::Mat lowerReflection = srcImg(lowerRect);
  cv::flip(lowerReflection, lowerReflection, 0);

  transpose(lowerReflection, lowerReflection);
  transpose(upperReflection, upperReflection);

  cv::flip(lowerReflection, lowerReflection, 1);
  cv::flip(upperReflection, upperReflection, 1);

  std::tuple<cv::Mat, cv::Mat> images(lowerReflection, upperReflection);
  return images;
}

cv::Mat Server::creatSkinMask(const cv::Mat &srcImg) {
  cv::Mat HSVMask, colrImg, nrgb, skinMask, RGBMask;

  cv::cvtColor(srcImg, colrImg, cv::COLOR_BGR2HSV);
  cv::inRange(colrImg, cv::Scalar(0, 51, 89), cv::Scalar(25, 174, 255), HSVMask);
  cv::GaussianBlur(HSVMask, HSVMask, cv::Size2i(3, 3), 0);
  
  normalize(srcImg, nrgb, 0, 1.0, cv::NORM_MINMAX);
  cv::inRange(nrgb, cv::Scalar(0, 0.28, 0.36), cv::Scalar(1.0, 0.363, 0.465), RGBMask);
  cv::GaussianBlur(RGBMask, RGBMask, cv::Size2i(3, 3), 0);
  bitwise_not(RGBMask, RGBMask);

  skinMask = HSVMask & RGBMask;
  return skinMask;
}

void Server::saveImg(cv::Mat img, std::string filename){
  if (cv::imwrite(filename, img)){
    // std::cout << "Saved image to: " << "build/" << filename << std::endl;
  }
  else {
    std::cout << "Error: could not save image to: " << "build/" << filename << "." << std::endl;
  }
}

void Server::sendMessage(int connfd, char *format){
  char sendBuff[1024];
  memset(sendBuff, '0', sizeof(sendBuff));
  snprintf(sendBuff, sizeof(sendBuff), format);
  write(connfd, sendBuff, strlen(sendBuff));
}

void Server::readVideoImgFromDisk(std::string filePath, int mode, int length, int start){
  m_video = cv::VideoCapture(filePath, mode);
  m_video.set(cv::CAP_PROP_POS_MSEC, start);
  cv::Mat tmpImg;

  if (!m_video.isOpened()){
    std::cout << "Warning: Failed to open video file." <<  std::endl;
  }

  for (unsigned int i = 0; i < length; i++){
    m_video.read(tmpImg);
    if(tmpImg.empty()) {
      std::cout << "Warning: read empty frame form video." << std::endl;
      break;
    }
    else{
      updateImageFeatures(tmpImg);
      creatDisplacementMap(tmpImg);
    }
  }

  m_video.release();
}

void Server::initializeMatchers(int window_size, int P1, int P2, int UniqRatio, int maxDiff, int specklSize, int filterCap, int mode){
  m_leftMatcher = cv::StereoSGBM::create(0, 16, window_size);
  m_leftMatcher->setP1(P1 * window_size * window_size);
  m_leftMatcher->setP2(P2 * window_size * window_size);
  m_leftMatcher->setUniquenessRatio(UniqRatio);
  m_leftMatcher->setDisp12MaxDiff(maxDiff);
  m_leftMatcher->setSpeckleWindowSize(specklSize);
  m_leftMatcher->setPreFilterCap(filterCap);
  m_leftMatcher->setMode(mode);

  m_rightMatcher = cv::ximgproc::createRightMatcher(m_leftMatcher);
}

void Server::initializeWLSFilter(int window_size, double lambda, double sigmeColor, int depthRadius){
  m_wlsFilter = cv::ximgproc::createDisparityWLSFilterGeneric(m_leftMatcher);
  m_wlsFilter->setDepthDiscontinuityRadius(depthRadius);
  m_wlsFilter->setLambda(lambda);
  m_wlsFilter->setSigmaColor(sigmeColor);
}

void Server::initializeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance){
  int min_disparity = matcher_instance->getMinDisparity();
  int num_disparities = matcher_instance->getNumDisparities();
  int block_size = matcher_instance->getBlockSize();

  int bs2 = block_size / 2;
  int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

  int xmin = maxD + bs2;
  int xmax = src_sz.width + minD - bs2;
  int ymin = bs2;
  int ymax = src_sz.height - bs2;

  cv::Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
  m_ROI = r;
}

void Server::updateImageFeatures(cv::Mat img){
  cv::Mat tmpImg = img.clone();
  m_imgFeatures.imgScale = 1.0f;
  
  cv::cvtColor(tmpImg, tmpImg, cv::COLOR_BGR2GRAY);
  m_imgFeatures.sift->cv::Feature2D::detect(tmpImg, m_imgFeatures.keyPoints);

  cv::drawKeypoints(tmpImg, m_imgFeatures.keyPoints, tmpImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  saveImg(tmpImg, "keypoints.jpg");
}