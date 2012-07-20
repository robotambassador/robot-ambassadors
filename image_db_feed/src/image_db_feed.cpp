#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <string>

class ImageFeed
{
  void publishImage(const std_msgs::StringConstPtr& msg);
  
  cv_bridge::CvImagePtr msg_ptr_;
  std::string path_;
  
  public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  ros::Subscriber next_image_sub_;
  
  ImageFeed(std::string path);
};

ImageFeed::ImageFeed(std::string path) : it_(nh_), path_(path)
{
  image_pub_ = it_.advertise("gscam/image_raw", 1, true);
  next_image_sub_ = nh_.subscribe("next_image", 1, &ImageFeed::publishImage, this);  
}

void ImageFeed::publishImage(const std_msgs::StringConstPtr& msg)
{
  std::string next_image = path_ + msg->data;
  cv::Mat im = cv::imread(next_image, 0);
  ROS_INFO("path: %s", next_image.c_str());
  if (!im.data)
  {
    ROS_ERROR("Couldn't load image");
    return;
  }
  msg_ptr_.reset (new cv_bridge::CvImage);
  msg_ptr_->image = im;
  msg_ptr_->encoding = "mono8";

  image_pub_.publish(msg_ptr_->toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_db_feed");
  
  ImageFeed image_feed(argv[1]);
    
  ros::spin();
  return 0;
}
