#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <cmath>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

#include "uv_msgs/ImageBoundingBox.h"
uv_msgs::ImageBoundingBox bbox_msg;

namespace enc = sensor_msgs::image_encodings;
using namespace message_filters;
using namespace std;
using namespace cv;

/* Global variables */
#define humanFrameID "human_detected"
#define fixedFrameID "kinect2_link"

ros::Subscriber img_sub;
ros::Publisher bbox_pub;

Size winStride = Size(8,8);
double groupThreshold = 2;
bool ValidPose = false;
Size padding = Size(4,4);
double scale = 1.2;

void interface(const sensor_msgs::ImageConstPtr& msg_rgb) {

  /* Local variables */
  vector<Rect> RectLst;
  HOGDescriptor Hog(Size(64,128), Size(16,16), Size(8,8), Size(8,8), 9, 1, -1, HOGDescriptor::L2Hys, 0.2, true, HOGDescriptor::DEFAULT_NLEVELS);
  Mat im_gray;

  /* Pre-processing */
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg_rgb,enc::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
  cvtColor(cv_ptr->image, im_gray, CV_BGR2GRAY );
  equalizeHist(im_gray, im_gray);
  
  /* Human detection */
  Hog.detectMultiScale(im_gray, RectLst, 0, winStride, padding, scale, groupThreshold, false);
  ros::Time rostime = ros::Time::now();
  if (RectLst.size() > 0)
    ValidPose = true;
  
  /* Publish bounding box*/
  if (ValidPose == true) {
    //bbox_msg.header.frame_id = fixedFrameID;
    //bbox_msg.header.stamp = rostime;
    bbox_msg.center.u = RectLst[0].x + RectLst[0].width/2;
    bbox_msg.center.v = RectLst[0].y + RectLst[0].height/2;
    bbox_msg.width = RectLst[0].width;
    bbox_msg.height = RectLst[0].height;
    bbox_msg.cornerPoints[0].u = RectLst[0].x;
    bbox_msg.cornerPoints[0].v = RectLst[0].y;
    bbox_msg.cornerPoints[1].u = RectLst[0].x + RectLst[0].width;
    bbox_msg.cornerPoints[1].v = RectLst[0].y;
    bbox_msg.cornerPoints[2].u = RectLst[0].x + RectLst[0].width;
    bbox_msg.cornerPoints[2].v = RectLst[0].y + RectLst[0].height;
    bbox_msg.cornerPoints[3].u = RectLst[0].x;
    bbox_msg.cornerPoints[3].v = RectLst[0].y + RectLst[0].height;
    bbox_pub.publish(bbox_msg);
    ValidPose = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "HOG_dection");
  ros::NodeHandle nh;

  img_sub = nh.subscribe("/kinect2/qhd/image_color_rect", 1000, interface); 
  bbox_pub = nh.advertise<uv_msgs::ImageBoundingBox>("/humanBBox", 20);
  
  while (ros::ok()) {
    ros::spinOnce();
  }
}
