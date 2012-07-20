#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_image_pub");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("goal_image", 1, true);
  
  cv::Mat im = cv::imread(argv[1], 0);
  if (!im.data)
  {
    ROS_ERROR("Couldn't load image");
    return -1;
  }
  cv_bridge::CvImagePtr msg(new cv_bridge::CvImage);
  msg->image = im;
  msg->encoding = "mono8";
  cv::imshow("home image", im);
  cv::waitKey(500);

  pub.publish(msg->toImageMsg());
  
  ROS_INFO("Published image");
  ros::spin();
  return 0;
}
