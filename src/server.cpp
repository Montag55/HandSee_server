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
    m_leftRightSwitch = 0;

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
  // saveImg(filter_Disp_vis, "disparity.jpg");
  // saveImg(masked, "disparityMasked.jpg");
  return masked;
}

void Server::splitImage(){
  /*
   * Note: Currently not entire image is used. 13 pixel rows are left out since
   * the mirror edge crosses all of these (as far as i can tell by using GIMP)
  **/

  /* Clone original image, otherwise overwrite */
  cv::Mat img = m_srcImg.clone();

  /* Scale image */
  cv::Size downScale(img.size().width * m_imgFeatures.imgScale, img.size().height * m_imgFeatures.imgScale);
  cv::resize(img, img, downScale);

  /* Create upper and lower image regions */
  // cv::Rect upperRect(0, 0, img.size().width, 617 * m_imgFeatures.imgScale);
  // cv::Rect lowerRect(0, 630 * m_imgFeatures.imgScale, img.size().width, img.size().height - (630 * m_imgFeatures.imgScale + 33 * m_imgFeatures.imgScale));
  cv::Rect upperRect(0, 0, img.size().width, 309 * m_imgFeatures.imgScale);
  cv::Rect lowerRect(0, 315 * m_imgFeatures.imgScale, img.size().width, img.size().height - (315 * m_imgFeatures.imgScale + 16 * m_imgFeatures.imgScale));

  /* Rotate/mirror images so the have same orientation */
  cv::Mat upperReflection = img(upperRect);
  cv::Mat lowerReflection = img(lowerRect);
  cv::flip(lowerReflection, lowerReflection, 0);

  transpose(lowerReflection, lowerReflection);
  transpose(upperReflection, upperReflection);
  
  cv::flip(lowerReflection, lowerReflection, 1);
  cv::flip(upperReflection, upperReflection, 1);

  if(m_leftRightSwitch == 1){
    m_right = upperReflection;
    m_left = lowerReflection;
  } 
  else{
    m_left = upperReflection;
    m_right = lowerReflection;
  }
}

void Server::createSkinMask() {
  /**
   * Creates a skin mask based on color in RGB and HSV color space.
   * this is later applied to the created disparity map to crop away
   * unintressting regions to reduce error.
   */

  cv::Mat HSVMask, colrImg;
  cv::Mat srcImg_L = m_left.clone();

  cv::cvtColor(srcImg_L, colrImg, cv::COLOR_BGR2HSV);
  cv::inRange(colrImg, cv::Scalar(0, 48, 80), cv::Scalar(20, 255, 255), HSVMask);
  //cv::inRange(colrImg, m_imgFeatures.minHSV, m_imgFeatures.maxHSV, HSVMask);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size2i(11, 11));
	cv::erode(HSVMask, HSVMask, kernel, cv::Point(-1, -1), 2);
  cv::dilate(HSVMask, HSVMask, kernel, cv::Point(-1, -1), 2);
  cv::GaussianBlur(HSVMask, HSVMask, cv::Size2i(3, 3), 0);

  // cv::rectangle(HSVMask, cv::Point(0, 0), cv::Point(HSVMask.cols - 1, HSVMask.rows - 1), cv::Scalar(255, 255, 255));
  for(int j = 0; j < HSVMask.rows; j++){
    uchar* current  = HSVMask.ptr<uchar>(j);
    for(int i = 0; i < HSVMask.cols; i += HSVMask.cols - 1){
        current[i] = 1;
        // std::cout << "j: " << j << "; i: " << i << std::endl;
    }
  }


  m_skinMask = HSVMask;
  // saveImg(m_skinMask, "skinMask.jpg");
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

  if (!m_video.isOpened()){
    std::cout << "Warning: Failed to open video file." <<  std::endl;
  }

  for (unsigned int i = 0; i < length; i++){
    m_video.read(m_srcImg);
    if(m_srcImg.empty()) {
      std::cout << "Warning: read empty frame form video." << std::endl;
      break;
    }
    else{
  
      splitImage();
      
      if(i == 0) {
        // calibrateColor();
        // std::cout << m_imgFeatures.minRGB << std::endl;
        // std::cout << m_imgFeatures.maxRGB << std::endl;
        updateImageFeatures();
      }

      createSkinMask();
      getContours();
      cv::cvtColor(m_srcImg, m_srcImg, cv::COLOR_BGR2GRAY);
      splitImage();

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
  std::cout << "Check" << std::endl;
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

  // if (m_imgFeatures.m_H1.at<double>(0, 2) * m_imgFeatures.m_H1.at<double>(1, 2) > 0 ||
  //     m_imgFeatures.m_H2.at<double>(0, 2) * m_imgFeatures.m_H2.at<double>(1, 2) > 0) {

  //   m_leftRightSwitch = (m_leftRightSwitch + 1) % 2;
  //   splitImage();
  //   updateImageFeatures();
  // }
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

  // saveImg(rectified1, "rectified1.jpg");
  // saveImg(rectified2, "rectified2.jpg");
  // saveImg(rectified3, "rectified3.jpg");
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
      cv::circle(mask, cv::Point(j, i), rad, cv::Scalar(255, 255, 255), -1, 8, 0);
    }
  }

  cv::Mat hsv = m_left.clone() & mask;
  cv::Mat rgb = m_left.clone() & mask;
  cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);

  cv::Mat rgbChannels [3];
  cv::split(rgb, rgbChannels);
  cv::Mat hsvChannels [3];
  cv::split(hsv, hsvChannels);
  cv::Mat maskArr[3];
  cv::split(mask, maskArr);

  cv::minMaxLoc(rgbChannels[0], &m_imgFeatures.minRGB[0], &m_imgFeatures.maxRGB[0], NULL, NULL, maskArr[0]);
  cv::minMaxLoc(rgbChannels[1], &m_imgFeatures.minRGB[1], &m_imgFeatures.maxRGB[1], NULL, NULL, maskArr[1]);
  cv::minMaxLoc(rgbChannels[2], &m_imgFeatures.minRGB[2], &m_imgFeatures.maxRGB[2], NULL, NULL, maskArr[2]);

  cv::minMaxLoc(hsvChannels[0], &m_imgFeatures.minHSV[0], &m_imgFeatures.maxHSV[0], NULL, NULL, maskArr[0]);
  cv::minMaxLoc(hsvChannels[1], &m_imgFeatures.minHSV[1], &m_imgFeatures.maxHSV[1], NULL, NULL, maskArr[1]);
  cv::minMaxLoc(hsvChannels[2], &m_imgFeatures.minHSV[2], &m_imgFeatures.maxHSV[2], NULL, NULL, maskArr[2]);
}

void Server::getContours(){

  cv::Mat src_gray = m_skinMask.clone();
  //cv::Mat src_gray = cv::imread("/home/lucas/Pictures/handMask.png", 1);
  
  std::vector < std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat canny_output;
  double thresh = 100;
  cv::Canny(src_gray, canny_output, thresh, thresh * 2, 3, true);
  cv::findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  // saveImg(canny_output, "edges.jpg");
  
  float maxArea = 0;
  int maxIdx = 0;

  for(unsigned int i = 0; i < contours.size(); i++){
    std::vector<cv::Point> cnt = contours[i];
    float area = cv::contourArea(cnt);
    // std::cout << area << std::endl;
    if(area > maxArea){
      maxArea = area;
      maxIdx = i;
    }
  }

  std::vector<int> hull;
  std::vector<cv::Vec4i> hullDefects;
  cv::Mat tmp = m_left.clone();
  if (contours.size() > 0){
    cv::convexHull( cv::Mat(contours[maxIdx]), hull, false, false);
    if(hull.size() > 3){
      cv::drawContours(tmp, contours, maxIdx, cv::Scalar(255, 255, 255), 10);
      cv::convexityDefects(cv::Mat(contours[maxIdx]), cv::Mat(hull), hullDefects);
      std::cout << findHand(contours[maxIdx], hull, hullDefects, tmp) << std::endl;
    }
  }
 
}

int Server::findHand(std::vector<cv::Point> contour, std::vector<int> hull, std::vector<cv::Vec4i> hullDefects, cv::Mat tmp){

  cv::Moments M = cv::moments(contour);
  cv::Point2f mc = cv::Point2f( M.m10/M.m00 , M.m01/M.m00 );
  cv::Mat tmp2 = tmp.clone();

  int cnt = 0;
  for(unsigned int i = 0; i < hullDefects.size(); i++){
    int s = (hullDefects[i])[0];
    int e = (hullDefects[i])[1];
    int f = (hullDefects[i])[2];
    int d = (hullDefects[i])[3];

    cv::Point start = contour[s];
    cv::Point end = contour[e];
    cv::Point far = contour[f];
    float angle = calcAngle(far, start, end);

    if(d > 1000 && angle <= CV_PI / 2){
      cnt += 1;
      cv::circle(tmp2, start, 8, cv::Scalar(255, 0, 0));
      cv::circle(tmp2, end, 8, cv::Scalar(0, 255, 0));
      cv::circle(tmp2, far, 8, cv::Scalar(0, 0, 255));
    }
  }
  cv::circle(tmp2, mc, 8, cv::Scalar(0, 0, 255), 8);
  //saveImg(tmp2, "tmp.jpg");
  return cnt;
}

float Server::calcAngle(cv::Point f, cv::Point s, cv::Point e){
  float a = sqrt((e.x - s.x) * (e.x - s.x) + (e.y - s.y) * (e.y - s.y));
  float b = sqrt((f.x - s.x) * (f.x - s.x) + (f.y - s.y) * (f.y - s.y));
  float c = sqrt((e.x - f.x) * (e.x - f.x) + (e.y - f.y) * (e.y - f.y));
  float angle = acos((b*b + c*c - a*a) / (2*b*c));
  return angle;
}


int Server::readStream(){
    cv::VideoCapture vcap;
    cv::Mat image;
    bool begin = true;

    const std::string videoStreamAddress = "rtmp://192.168.2.119/app/live"; 

    //open the video stream and make sure it's opened
    if(!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }
    std::cout << "Check1" << std::endl;
    cv::namedWindow("Output Window");
    std::cout << "Check2" << std::endl;
    for(;;) {
        if(!vcap.read(m_srcImg)) {
            std::cout << "No frame" << std::endl;
            cv::waitKey();
        }
        //m_srcImg = cv::resize(m_srcImg, 50);
        splitImage();
        if(begin){
          std::cout << "Check3" << std::endl;
          updateImageFeatures();
          begin = false;
          std::cout << "Check4" << std::endl;
        }

        createSkinMask();
        getContours();
        cv::cvtColor(m_srcImg, m_srcImg, cv::COLOR_BGR2GRAY);
        splitImage();

        rectification();
        cv::Mat result = createDisplacementMap();

        cv::imshow("Output Window", result);
        if(cv::waitKey(1) >= 0) break;
    }   
}
