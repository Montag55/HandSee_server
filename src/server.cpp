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

cv::Mat Server::createDisplacementMap(){

  /* vars for use */
  cv::Mat disp_left, disp_right, filteredDisp, raw_Disp_vis, filter_Disp_vis;
  
  /* initiaize RIO if not yet done */
  if (m_initializationStatus == false){
    initializeROI(m_left.size(), m_leftMatcher);
    m_initializationStatus = true;
  }
  
  /* compute left and right disparity matcher */
  m_leftMatcher->compute(m_left, m_right, disp_left);
  m_rightMatcher->compute(m_right, m_left, disp_right);
  disp_left.convertTo(disp_left, CV_16S);
  disp_right.convertTo(disp_right, CV_16S);

  /* compute disparity map */
  m_wlsFilter->filter(disp_left, m_left, filteredDisp, disp_right, m_ROI, m_right);
  cv::ximgproc::getDisparityVis(filteredDisp, filter_Disp_vis, 2.0f);

  /* apply skin mask on disparity map*/
  cv::Mat masked;
  cv::bitwise_and(filter_Disp_vis, filter_Disp_vis, masked, m_skinMask);
  cv::normalize(masked, masked, 255.0f, 0, cv::NORM_MINMAX);
  //cv::absdiff(filter_Disp_vis, cv::Scalar(255, 255, 255), masked);
  saveImg(filter_Disp_vis, "disparity.jpg");
  saveImg(masked, "disparityMasked.jpg");
  return masked;
}

void Server::splitImage(const cv::Mat& inputIMG){
  /*
   * Note: Currently not entire image is used. 13 pixel rows are left out since
   * the mirror edge crosses all of these (as far as i can tell by using GIMP)
  **/

  /* Clone original image, otherwise overwrite */
  cv::Mat img = inputIMG.clone();

  /* Scale image */
  cv::Size downScale(img.size().width * m_imgFeatures.imgScale, img.size().height * m_imgFeatures.imgScale);
  cv::resize(img, img, downScale);

  /* Create upper and lower image regions */
  cv::Rect upperRect(0, 0, img.size().width, 617 * m_imgFeatures.imgScale);
  cv::Rect lowerRect(0, 630 * m_imgFeatures.imgScale, img.size().width, img.size().height - (630 * m_imgFeatures.imgScale + 33 * m_imgFeatures.imgScale));

  /* Rotate/mirror images so the have same orientation */
  cv::Mat upperReflection = img(upperRect);
  cv::Mat lowerReflection = img(lowerRect);
  cv::flip(lowerReflection, lowerReflection, 0);

  transpose(lowerReflection, lowerReflection);
  transpose(upperReflection, upperReflection);
  
  // cv::flip(lowerReflection, lowerReflection, 1);
  // cv::flip(upperReflection, upperReflection, 1);

  m_right = upperReflection;
  m_left = lowerReflection;
  
  // m_left = upperReflection;
  // m_right = lowerReflection;
}

void Server::createSkinMask() {
  /**
   * Creates a skin mask based on color in RGB and HSV color space.
   * this is later applied to the created disparity map to crop away
   * unintressting regions to reduce error.
   */

  cv::Mat HSVMask, colrImg, nrgb, RGBMask;
  cv::Mat srcImg_L = m_left.clone();

  cv::cvtColor(srcImg_L, colrImg, cv::COLOR_BGR2HSV);
  // cv::inRange(colrImg, cv::Scalar(0, 51, 89), cv::Scalar(25, 174, 255), HSVMask);
  cv::inRange(colrImg, cv::Scalar(0, 50, 20), cv::Scalar(255, 255, 255), HSVMask);
  // cv::inRange(colrImg, m_imgFeatures.minHSV, m_imgFeatures.maxHSV, HSVMask);
  cv::GaussianBlur(HSVMask, HSVMask, cv::Size2i(3, 3), 0);
  
  normalize(srcImg_L, nrgb, 0, 1.0, cv::NORM_MINMAX);
  cv::inRange(nrgb, cv::Scalar(0, 0.28, 0.36), cv::Scalar(1.0, 0.363, 0.465), RGBMask);
  // cv::inRange(nrgb, m_imgFeatures.minRGB, m_imgFeatures.maxRGB, RGBMask);
  cv::GaussianBlur(RGBMask, RGBMask, cv::Size2i(3, 3), 0);
  bitwise_not(RGBMask, RGBMask);

  m_skinMask = HSVMask & RGBMask;
}

void Server::saveImg(cv::Mat img, std::string filename){
  /**
   * Simple convinience function. Saves the given input matix as a
   * picture. Root directory: build/
   */

  if (!cv::imwrite(filename, img)){
    std::cout << "Error: could not save image to: " << "build/" << filename << "." << std::endl;
    return;
  }
}

void Server::sendMessage(int connfd, char *format){
  char sendBuff[1024];
  memset(sendBuff, '0', sizeof(sendBuff));
  snprintf(sendBuff, sizeof(sendBuff), format);
  write(connfd, sendBuff, strlen(sendBuff));
}

void Server::readVideoImgFromDisk(std::string filePath, int mode, int length, int start){
  /**
   * Function that reads all pictures in a directory in order.
   * Naming convention - frame followed by number. Numbering 
   * has t start at zero. 
   * Starting frames and amount of frames (reding length) is
   * specifiable.
   */

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
  
      splitImage(tmpImg);
      
      if(i == 0) {
        calibrateColor();
        updateImageFeatures();
      }

      createSkinMask();  
      cv::cvtColor(tmpImg, tmpImg, cv::COLOR_BGR2GRAY);
      splitImage(tmpImg);

      rectification();
      cv::Mat result = createDisplacementMap();
  
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
  /**
   * Initializes region of intrest (ROI) derived from the matcher instance.
   * This ROI, after initialization, is then used for further disparity map
   * creation. This however is OPTIONAL. Disparity map can also be comuted 
   * without a ROI.
   */

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

void Server::updateImageFeatures(){
  /**
   * https://stackoverflow.com/questions/45855725/does-the-stereobm-class-in-opencv-do-rectification-of-the-input-images-or-frames
   * This method is supposed to be called every once in a while to
   * the update homography matrix m_H1 m_H2 and fundamental matrix F for 
   * rectification. The Sift algorithm along with closest point matching 
   * is used to detect corresponding points in the images.
   */

  cv::Mat left = m_left.clone();
  cv::Mat right = m_right.clone();
  
  double max_dist = 0;
  double min_dist = 100;
  cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  cv::Ptr<cv::Feature2D> fd = cv::xfeatures2d::SIFT::create();
  cv::Mat descriptors_1, descriptors_2;
  cv::BFMatcher matcher(cv::NORM_L2, true);
  std::vector<cv::DMatch> matches;
  std::vector<cv::DMatch> good_matches;
  std::vector<cv::Point2f> imgpts1, imgpts2;
  std::vector<uchar> status;

  f2d->detect(left, keypoints_1);
  f2d->detect(right, keypoints_2);

  fd->compute(left, keypoints_1, descriptors_1);
  fd->compute(right, keypoints_2, descriptors_2);

  matcher.match(descriptors_1, descriptors_2, matches);

  for (int i = 0; i < matches.size(); i++) {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  for (int i = 0; i < matches.size(); i++) {
    if (matches[i].distance <= std::max(4.5 * min_dist, 0.02)) {
      good_matches.push_back(matches[i]);
      imgpts1.push_back(keypoints_1[matches[i].queryIdx].pt);
      imgpts2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
  }

  m_imgFeatures.m_F = findFundamentalMat(imgpts1, imgpts2, cv::FM_RANSAC, 3.0, 0.99, status);
  cv::stereoRectifyUncalibrated(imgpts1, imgpts1, m_imgFeatures.m_F, left.size(), m_imgFeatures.m_H1, m_imgFeatures.m_H2);
}

void Server::rectification(){
  /**
   * https://stackoverflow.com/questions/45855725/does-the-stereobm-class-in-opencv-do-rectification-of-the-input-images-or-frames
   * Rectification of images to come as close as possible to stereo normal case
   * https://books.google.de/books?id=seAgiOfu2EIC&lpg=PA415&ots=hUJ79hbCRc&dq=opencv%20rectification%20different%20orientation&hl=de&pg=PA415#v=onepage&q=opencv%20rectification%20different%20orientation&f=false
   */

  cv::Mat left = m_left.clone();
  cv::Mat right = m_right.clone();
  cv::Mat skinMask = m_skinMask.clone();

  cv::Mat rectified1(left.size(), left.type());
  cv::warpPerspective(left, rectified1, m_imgFeatures.m_H1, left.size());

  cv::Mat rectified2(right.size(), right.type());
  cv::warpPerspective(right, rectified2, m_imgFeatures.m_H2, right.size());

  cv::Mat rectified3(skinMask.size(), skinMask.type());
  cv::warpPerspective(skinMask, rectified3, m_imgFeatures.m_H1, skinMask.size());


  m_left = rectified1;
  m_right = rectified2;
  m_skinMask = rectified3;

  saveImg(rectified1, "rectified1.jpg");
  saveImg(rectified2, "rectified2.jpg");
  saveImg(rectified3, "rectified3.jpg");
}

cv::Point Server::retrieveOriginalImgPointPos(cv::Point warpedPoint){
  cv::Mat img = m_left.clone();
  cv::circle(img, cv::Point((int)(img.cols / 2), (int)(img.rows / 2)), 1, cv::Scalar(0, 0, 255), 10);
  cv::Mat test_rect(img.size(), img.type());
  cv::warpPerspective(img, test_rect, m_imgFeatures.m_H1, img.size());
  cv::warpPerspective(test_rect, test_rect, m_imgFeatures.m_H1.inv(), test_rect.size());

  return cv::Point(-1, -1);
}

void Server::calibrateColor(){
  
  int stepSize = 4;
  int rad = 20;
  cv::Mat mask = cv::Mat::zeros(m_left.size(), m_left.type());

  int x_step = mask.cols / stepSize;
  int y_step = mask.rows / stepSize;

  for (int i = y_step; i < mask.rows - (y_step - 1); i += y_step) {
    for (int j = x_step; j < mask.cols - (x_step - 1); j += x_step) {

      cv::Point center(j, i);
      cv::circle(mask, center, rad, cv::Scalar(255, 255, 255), -1, 8, 0);
    }
  }

  cv::Mat hsv = m_left.clone() & mask;
  cv::Mat rgb = m_left.clone() & mask;
  cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);

  cv::Mat rgbChannels [3];
  cv::split(rgb, rgbChannels);
  cv::Mat hsvChannels [3];
  cv::split(hsv, hsvChannels);

  cv::minMaxLoc(rgbChannels[0], &m_imgFeatures.minRGB[0], &m_imgFeatures.maxRGB[0]);
  cv::minMaxLoc(rgbChannels[1], &m_imgFeatures.minRGB[1], &m_imgFeatures.maxRGB[1]);
  cv::minMaxLoc(rgbChannels[2], &m_imgFeatures.minRGB[2], &m_imgFeatures.maxRGB[2]);
  cv::normalize(m_imgFeatures.minRGB, m_imgFeatures.minRGB);
  cv::normalize(m_imgFeatures.maxRGB, m_imgFeatures.maxRGB);

  cv::minMaxLoc(hsvChannels[0], &m_imgFeatures.minHSV[0], &m_imgFeatures.maxHSV[0]);
  cv::minMaxLoc(hsvChannels[1], &m_imgFeatures.minHSV[1], &m_imgFeatures.maxHSV[1]);
  cv::minMaxLoc(hsvChannels[2], &m_imgFeatures.minHSV[2], &m_imgFeatures.maxHSV[2]); 
}
