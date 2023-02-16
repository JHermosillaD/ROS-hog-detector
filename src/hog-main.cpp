#include <ros/ros.h>
#include <iostream>
#include <iomanip>

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

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

namespace enc = sensor_msgs::image_encodings;
using namespace message_filters;
using namespace std;
using namespace cv;

ros::Publisher marker_pub;
ros::Publisher pt_pub;

ros::Subscriber pcd_sub;
ros::Subscriber sub;

void add_marker(const float xx, const float yy, const ros::Time& rostime) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "human_detected";
  marker.header.stamp = rostime;
  marker.ns = "basic_shapes";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = xx;
  marker.pose.position.y = yy;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 0.01;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
        return ;
      }
      sleep(1);
    }
    marker_pub.publish(marker);
}

void add_position(const vector<Rect> found, const sensor_msgs::PointCloud2ConstPtr& pCloud, const ros::Time& rostime) {
  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;
  int u_px = found[0].x + found[0].width/2;
  int v_px = found[0].y + found[0].height/2;
  int arrayPosition = v_px*pCloud->row_step + u_px*pCloud->point_step;
  int arrayPosX = arrayPosition + pCloud->fields[0].offset;
  int arrayPosY = arrayPosition + pCloud->fields[1].offset;
  int arrayPosZ = arrayPosition + pCloud->fields[2].offset;

  memcpy(&X, &pCloud->data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud->data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud->data[arrayPosZ], sizeof(float));

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped humanPose;
    
  transformStamped.header.stamp = rostime;
  transformStamped.header.frame_id = pCloud->header.frame_id;
  transformStamped.child_frame_id = "human_detected";
  transformStamped.transform.translation.x = X;
  transformStamped.transform.translation.y = Y;
  transformStamped.transform.translation.z = Z;
  tf2::Quaternion q;
  q.setRPY(-M_PI/2.0, M_PI/2.0, M_PI);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
  humanPose.header.stamp = rostime;
  humanPose.header.frame_id ="human_detected";
  humanPose.pose.position.x = X; //red
  humanPose.pose.position.y = Y; //green
  humanPose.pose.position.z = Z; //blue
  pt_pub.publish(humanPose);
  add_marker(X, Y, rostime);
}

void RGBDcallback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::PointCloud2ConstPtr& pCloud) {
  cv_bridge::CvImageConstPtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvShare(msg_rgb,enc::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  HOGDescriptor hog_;
  vector<Rect> found;
  Mat im_gray;

  hog_.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
  cvtColor(cv_ptr->image, im_gray, CV_BGR2GRAY );
  equalizeHist(im_gray, im_gray);
  hog_.detectMultiScale(im_gray, found, 0, Size(8,8), Size(), 1.1, 2, false);
  if (found.size() > 0) {
    ros::Time rostime = ros::Time::now();
    add_position(found, pCloud, rostime);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Human_dection");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcd_sub(nh, "/camera/depth_registered/points", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), rgb_sub, pcd_sub);
  sync.registerCallback(boost::bind(&RGBDcallback, _1, _2));

  pt_pub = nh.advertise<geometry_msgs::PoseStamped>("hog/pose", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("my_visualization_marker", 1);
  
  while (ros::ok()) {
    ros::spinOnce();
  }
}
