/************ Rotate image by 180 degrees ************/

#include <iostream>
using namespace std;

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


image_transport::Publisher pub_rotated_image;


void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  // ROS_INFO_STREAM("encoding: " << msg->encoding);  // bgr8

  // /************ Rotate by 180 degrees ************/
  cv::Mat rotated_image;
  cv::flip(cv_in_ptr->image, rotated_image, -1);

  std_msgs::Header header;
  header.stamp    = msg->header.stamp;     //current_frame_time_;
  header.frame_id = msg->header.frame_id;  //map_frame_id_param_;
  const sensor_msgs::ImagePtr rotated_image_msg = cv_bridge::CvImage(header, "bgr8", rotated_image).toImageMsg();

  pub_rotated_image.publish(rotated_image_msg);
}


int main(int argc, char * argv[])
{
  // if (argc < 2)
  // {
  //   ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
  //   exit(-1);
  // }

  std::string topic_root = "/shihushan";
  std::string topic_org_image = "/hikrobot_camera/rgb";

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMRotateImage");
  }
  ros::NodeHandle node;

  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe(topic_org_image, 1, image_callback);

  pub_rotated_image = it.advertise (topic_root + "/rotated_image", 1);

  ros::spin();

  return 0;
}
