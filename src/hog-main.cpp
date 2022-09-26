#include <ros/ros.h>
#include <iostream>
#include <iomanip>

#include <image_transport/image_transport.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;
using namespace message_filters;

void RGBDcallback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth) {  
  cv_bridge::CvImagePtr rgb_ptr;
  cv_bridge::CvImagePtr dpt_ptr;
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
