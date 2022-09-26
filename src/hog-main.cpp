#include <ros/ros.h>
#include <iostream>

#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;
using namespace message_filters;

class Detector {
  enum Mode {Default, Daimler} m;
  HOGDescriptor hog, hog_d;
public:
  Detector() : m(Default), hog(), hog_d(Size(48, 96), Size(16, 16), Size(8, 8), Size(8, 8), 9) {
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    hog_d.setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
  }
  void toggleMode() { m = (m == Default ? Daimler : Default); }
  string modeName() const { return (m == Default ? "Default" : "Daimler"); }
  vector<Rect> detect(InputArray img) {
    vector<Rect> found;
    if (m == Default)
      hog.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, false);
    else if (m == Daimler)
      hog_d.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, true);
    return found;
  }
  void adjustRect(Rect & r) const {
    r.x += cvRound(r.width*0.1);
    r.width = cvRound(r.width*0.8);
    r.y += cvRound(r.height*0.07);
    r.height = cvRound(r.height*0.8);
  }
};

void RGBDcallback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth) {  
  cv_bridge::CvImagePtr rgb_ptr;
  cv_bridge::CvImagePtr dpt_ptr;
  
  try { 
    rgb_ptr = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert '%s' format", e.what());
  }
 
  try{
    dpt_ptr = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert '%s' format", e.what());
  }

  rgb_frame = rgb_ptr->image;
  dpt_frame = dpt_ptr->image;
  Mat rgb_detected_frame;
  Mat dpt_detected_frame;
  Detector detector; 
  vector<Rect> found = detector.detect(rgb_frame);
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Body_dection");
  ros::NodeHandle n;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, "/camera/rgb/image_raw", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&RGBDcallback, _1, _2));

  ros::spin();
  return 0;
}
