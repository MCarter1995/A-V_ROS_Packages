#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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


    pub_right.publish(right_img);
    pub_left.publish(left_img);
    ros::spinOnce();

  }
}