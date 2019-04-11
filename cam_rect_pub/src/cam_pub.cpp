#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <sstream>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;


template<typename T>
T getParam(ros::NodeHandle n, const std::string& name, const T& defaultValue)
{
  T v;
  if (n.getParam(name, v))
  {
    ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
  {
    ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  }
  return defaultValue;
}

int fillCameraInfoMessage(const std::string & fileName, sensor_msgs::CameraInfo &msg, cv::Mat )
{
  int ii;
  cv::Mat KK(3, 3, CV_64FC1);
  cv::Mat DD(5, 1, CV_64FC1);
  cv::FileStorage fs;

  //open file
  fs.open(fileName, cv::FileStorage::READ);
  if ( !fs.isOpened() )
  {
    std::cout << "WARNING: Camera Calibration File " << fileName << " Not Found." << std::endl;
    std:: cout << "WARNING: camera_info topic will not provide right data" << std::endl;
    return BFLY_ERROR;
  }

  //fill static part of the message
  msg.header.frame_id = "bfly_camera";
  msg.binning_x = 0;
  msg.binning_x = 0;
  msg.roi.width = 0;
  msg.roi.height = 0;
  for (ii = 0; ii < 9; ii++) msg.R[ii] = 0;
  msg.width = (int)fs["image_Width"];
  msg.height = (int)fs["image_Height"];
  fs["Camera_Matrix"] >> KK;
  for (ii = 0; ii < 9; ii++) msg.K[ii] = KK.at<double>(ii);
  msg.distortion_model = "plumb_bob";
  fs["Distortion_Coefficients"] >> DD;
  for (ii = 0; ii < 5; ii++) msg.D.push_back(DD.at<double>(ii));

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_publisher");
  ros::NodeHandle nh;

  int cam_right;
  int cam_left;
  int cam_width;
  int cam_height;

  string intrinsic_filename;

  nh.param<string>("cam_intrinsic_loc", intrinsic_filename, "empty");
  nh.param<int>("cam_right", cam_right, 1);
  nh.param<int>("cam_left", cam_left, 2);
  nh.param<int>("cam_width", cam_width, 1280);
  nh.param<int>("cam_height", cam_height, 720);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_right = it.advertise("cameras/image_rect_right", 1);
  image_transport::Publisher pub_left = it.advertise("cameras/image_rect_left", 1);

  ros::Publisher pub_info_right = nh.advertise<sensor_msgs::CameraInfo>("cameras/image_rect_right/camera_info", 100);
  ros::Publisher pub_info_left = nh.advertise<sensor_msgs::CameraInfo>("cameras/image_rect_left/camera_info", 100);

  sensor_msgs::CameraInfo cameraInfoMessage_right;
  sensor_msgs::CameraInfo cameraInfoMessage_right;

  bool rect = false;

  Mat rmap[2][2];
  if ( intrinsic_filename != "empty" ) {
    // reading intrinsic parameters
    FileStorage fs(intrinsic_filename, FileStorage::READ);
    if (!fs.isOpened()) {
      printf("Failed to open file %s\n", intrinsic_filename.c_str());
      return -1;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    cameraInfoMessage_right.frame_id = "right_camera";
    cameraInfoMessage_right.binning_x = 0;
    cameraInfoMessage_right.binning_y = 0;
    cameraInfoMessage_right.roi.width = 0;
    cameraInfoMessage_right.roi.height = 0;
    cameraInfoMessage_right.width = cam_width;
    cameraInfoMessage_right.height = cam_height;
    for (int i = 0; i < 9; i++) {cameraInfoMessage_right.R[i] = 0;}
    for (int i = 0; i < 9; i++) {cameraInfoMessage_right.K[i] = M1.at<double>(i);}
    for (int i = 0; i < 5; i++) {cameraInfoMessage_right.D.push_back(D1.at<double>(i));}
    cameraInfoMessage_right.distortion_model = "plumb_bob";

    cameraInfoMessage_left.frame_id = "left_camera";
    cameraInfoMessage_left.binning_x = 0;
    cameraInfoMessage_left.binning_y = 0;
    cameraInfoMessage_left.roi.width = 0;
    cameraInfoMessage_left.roi.height = 0;
    cameraInfoMessage_left.width = cam_width;
    cameraInfoMessage_left.height = cam_height;
    for (int i = 0; i < 9; i++) {cameraInfoMessage_left.R[i] = 0;}
    for (int i = 0; i < 9; i++) {cameraInfoMessage_left.K[i] = M2.at<double>(i);}
    for (int i = 0; i < 5; i++) {cameraInfoMessage_left.D.push_back(D2.at<double>(i));}
    cameraInfoMessage_left.distortion_model = "plumb_bob";

    Mat R1 = Mat_<double>::eye(3, 3);
    Mat R2 = Mat_<double>::eye(3, 3);

    Size imageSize;
    imageSize.width = cam_width;
    imageSize.height = cam_height;

    //Precompute maps for cv::remap()
    fisheye::initUndistortRectifyMap(M1, D1, R1, M1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    fisheye::initUndistortRectifyMap(M2, D2, R2, M2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    rect = true;
  } else {
    ROS_DEBUG("Unable to read intrinsics file.");
  }

  VideoCapture right(cam_right);
  VideoCapture left(cam_left);

  right.set(CV_CAP_PROP_FRAME_WIDTH, cam_width);
  right.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);

  left.set(CV_CAP_PROP_FRAME_WIDTH, cam_width);
  left.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);

  right.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
  left.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));

  Mat r_img, l_img;
  Mat r_rimg, l_rimg;

  sensor_msgs::ImagePtr right_img;
  sensor_msgs::ImagePtr left_img;

  while (nh.ok()) {

    right.grab();
    left.grab();

    right.retrieve(r_img);
    left.retrieve(l_img);

    if (rect) {
      remap(r_img, r_rimg, rmap[0][0], rmap[0][1], INTER_LINEAR);
      remap(l_img, l_rimg, rmap[1][0], rmap[1][1], INTER_LINEAR);
      right_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", r_rimg).toImageMsg();
      left_img  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", l_rimg).toImageMsg();
    } else {
      right_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", r_img).toImageMsg();
      left_img  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", l_img).toImageMsg();
    }
    cameraInfoMessage_right.header.stamp = ros::Time::now();
    cameraInfoMessage_left.header.stamp = ros::Time::now();
    pub_info_right.publish(cameraInfoMessage_right);
    pub_info_left.publish(cameraInfoMessage_left);

    pub_right.publish(right_img);
    pub_left.publish(left_img);
    ros::spinOnce();

  }
}