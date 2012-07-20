#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <visual_homing/homingtools.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

class DescentImageDistance
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber goal_image_sub_;
  
  ros::Publisher rms_pub_;
  ros::Publisher cmd_pub_;
  ros::ServiceServer home_srv_;
  
  Mat home_;
  cv_bridge::CvImagePtr cv_image_;
  
  public:
  DescentImageDistance();  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void goalImageCallback(const sensor_msgs::ImageConstPtr& msg);
  bool saveHome();
};

DescentImageDistance::DescentImageDistance()
    : it_(nh_)
{
  goal_image_sub_ = it_.subscribe("goal_image", 1, &DescentImageDistance::goalImageCallback, this);
  image_sub_ = it_.subscribe("/gscam/image_raw", 1, &DescentImageDistance::imageCallback, this);
  
  rms_pub_ = nh_.advertise<std_msgs::Float32>("rms_error", 1, true);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  home_srv_ = nh_.advertiseService("save_home", saveHome);
}

void DescentImageDistance::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!home_.data) 
  {
    ROS_ERROR("No home image found");
    return;
  }
  try
  {
    cv_image_ = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  float rms_error = HomingTools::rms(cv_image_->image, home_);
  std_msgs::Float32 rms_msg;
  rms_msg.data = rms_error;
  
  rms_pub_.publish(rms_msg);
  ROS_DEBUG("Published rms as %f", rms_error);
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
  home_ = cv_image_->image;
  ROS_INFO("Copied home image");  
}  

bool DescentImageDistance::saveHome()
{
  home_ = cv_image_->image;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "did_homing");
  DescentImageDistance did;
  ros::spin();
  return 0;
}
