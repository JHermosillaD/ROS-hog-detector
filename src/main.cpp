#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <hog/ImageBoundingBox.h>

using namespace std;
using namespace cv;

string image_topic;

class HogDetector {

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher bbox_pub_;  
  
public:
  HOGDescriptor Hog;
  HogDetector()
    : it_(nh_) {
    image_sub_ = it_.subscribe(image_topic, 1, &HogDetector::cameraCallback, this);
    image_pub_ = it_.advertise("/hog/image", 1);
    bbox_pub_ = nh_.advertise<hog::ImageBoundingBox>("/hog/bounding_box",1000);
    Hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
  }

  void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat img_gray;
    cvtColor(cv_ptr->image, img_gray, CV_BGR2GRAY );
    equalizeHist(img_gray, img_gray);
    vector<Rect> bodies;
    Hog.detectMultiScale(img_gray, bodies, 0.0, cv::Size(8, 8), cv::Size(0,0), 1.05, 4);
    hog::ImageBoundingBox bbox_msg;
    if (bodies.size() > 0) {
      for(unsigned i=0; i<bodies.size(); i++) {
	      Point left_corner(bodies[i].x, bodies[i].y);
	      Point right_corner(bodies[0].x + bodies[0].width, bodies[0].y + bodies[0].height);
	      rectangle(cv_ptr->image,left_corner,right_corner,Scalar(255,0,0), 2, LINE_8);
      }
      bbox_msg.center.u = bodies[0].x + bodies[0].width/2;
      bbox_msg.center.v = bodies[0].y + bodies[0].height/2;
      bbox_msg.width = bodies[0].width;
      bbox_msg.height = bodies[0].height;
      bbox_msg.cornerPoints[0].u = bodies[0].x;
      bbox_msg.cornerPoints[0].v = bodies[0].y;
      bbox_msg.cornerPoints[1].u = bodies[0].x + bodies[0].width;
      bbox_msg.cornerPoints[1].v = bodies[0].y;
      bbox_msg.cornerPoints[2].u = bodies[0].x + bodies[0].width;
      bbox_msg.cornerPoints[2].v = bodies[0].y + bodies[0].height;
      bbox_msg.cornerPoints[3].u = bodies[0].x;
      bbox_msg.cornerPoints[3].v = bodies[0].y + bodies[0].height;
    }
    bbox_pub_.publish(bbox_msg);
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main (int argc, char** argv) {
  ros::init(argc, argv, "hog_detector");
  ros::NodeHandle nh;
  nh.getParam("/camera_topic", image_topic);
  HogDetector ic;
  ros::spin ();
  return 0;
}