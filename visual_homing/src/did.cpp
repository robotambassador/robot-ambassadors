#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include "rms.cpp"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

class DescentImageDistance
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber goal_image_sub_;
  
  ros::Publisher rms_pub_;
  
  RMS_Error rms;
  Mat home;
  
  public:
  DescentImageDistance();  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void goalImageCallback(const sensor_msgs::ImageConstPtr& msg);
};

DescentImageDistance::DescentImageDistance()
    : it_(nh_)
{
  image_sub_ = it_.subscribe("/gscam/image_raw", 1, &DescentImageDistance::imageCallback, this);
  goal_image_sub_ = it_.subscribe("goal_image", 1, &DescentImageDistance::goalImageCallback, this);
  rms_pub_ = nh_.advertise<std_msgs::Float32>("rms_error", 1);
}

void DescentImageDistance::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  float rms_error = rms.rms(cv_image->image, home);
  std_msgs::Float32 rms_msg;
  rms_msg.data = rms_error;
  
  rms_pub_.publish(rms_msg);

}

void DescentImageDistance::goalImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  home = cv_image->image;
}    

int main(int argc, char** argv)
{
  ros::init(argc, argv, "did_homing");
  DescentImageDistance did;
  ros::spin();
  return 0;
}
