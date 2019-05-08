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



int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_publisher");
  ros::NodeHandle nh;

  int cam_right;
  int cam_width;
  int cam_height;

  string intrinsic_filename;

  nh.param<string>("cam_intrinsic_loc", intrinsic_filename, "empty");
  nh.param<int>("cam_right", cam_right, 1);
  nh.param<int>("cam_width", cam_width, 1280);
  nh.param<int>("cam_height", cam_height, 720);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_right = it.advertise("cameras/image_rect_right", 1);

  ros::Publisher pub_info_right = nh.advertise<sensor_msgs::CameraInfo>("cameras/image_rect_right/camera_info", 100);

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

    Mat M1, D1;
    fs["M1"] >> M1;
    fs["D1"] >> D1;

    cameraInfoMessage_right.header.frame_id = "right_cam";
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

    Mat R1 = Mat_<double>::eye(3, 3);

    Size imageSize;
    imageSize.width = cam_width;
    imageSize.height = cam_height;

    //Precompute maps for cv::remap()
    fisheye::initUndistortRectifyMap(M1, D1, R1, M1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);

    rect = true;
  } else {
    ROS_DEBUG("Unable to read intrinsics file.");
  }

  VideoCapture right(cam_right);

  right.set(CV_CAP_PROP_FRAME_WIDTH, cam_width);
  right.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);


  right.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
  Mat r_img, r_rimg;

  sensor_msgs::ImagePtr right_img;

  while (nh.ok()) {

    right.grab();

    right.retrieve(r_img);

    if (rect) {
      remap(r_img, r_rimg, rmap[0][0], rmap[0][1], INTER_LINEAR);
      right_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", r_rimg).toImageMsg();
    } else {
      right_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", r_img).toImageMsg();
    }
    cameraInfoMessage_right.header.stamp = ros::Time::now();
    pub_info_right.publish(cameraInfoMessage_right);

    pub_right.publish(right_img);
    ros::spinOnce();

  }
}