#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <string>

class ImageFeed
{
  public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher goal_pub_;
  ros::Subscriber sub_;
  
  ImageFeed(std::string path);
  
  private:
  void commandCallback(const geometry_msgs::Twist& cmd);
  void publishImage();
  
  cv_bridge::CvImagePtr msg_ptr_;
  int x, y;
  double angle;
  std::string path_;
};

ImageFeed::ImageFeed(std::string path) : it_(nh_)
{
  image_pub_ = it_.advertise("gscam/image_raw", 1);
  goal_pub_ = it_.advertise("goal_image", 1);
  sub_ = nh_.subscribe("cmd_vel", 1, &ImageFeed::commandCallback, this);
  
  path_ = path;
  
  cv::namedWindow("Image",CV_WINDOW_AUTOSIZE);
  
  x = 0;
  y = 0;
  angle = 0;  
  
  if (nh_.ok())
  {
    publishImage();
    goal_pub_.publish(msg_ptr_->toImageMsg());
  }
    
  //Republish command if no command recieved after 10s
  /*ros::Rate loop_rate(0.1);
  while (nh_.ok()) {
    pub_.publish(msg_ptr_->toImageMsg());
    ros::spinOnce();
    loop_rate.sleep();
  }*/
  
}

void ImageFeed::commandCallback(const geometry_msgs::Twist& cmd)
{
  if (cmd.linear.x > 0) {
    if (angle > -0.785398163 && angle < 0.785398163)
      x+=1;
    else if (angle > 2.35619449 || angle < -2.35619449)
      x-=1;
    else if (angle > 0.785398163 && angle < 2.35619449)
      y+=1;
    else if (angle > -2.35619449 && angle < -0.785398163)
      y-=1;
  }
  
  angle += cmd.angular.z;
  if (angle > 3.14)
    angle -= 6.28;
  else if (angle <= -3.14)
    angle += 6.28;
  
  publishImage();
  
  ROS_INFO("received cmd, new pos x: %d, y: %d, angle: %f", x, y, angle);
}

void ImageFeed::publishImage()
{
  std::stringstream sstm;
  sstm << path_ << x << "_" << y << ".ppm";
  std::string next_image = sstm.str();
  cv::Mat im = cv::imread(next_image, 0);
  ROS_INFO("path: %s", next_image.c_str());
  if (!im.data)
  {
    ROS_ERROR("Couldn't load image");
    return;
  }
  msg_ptr_.reset (new cv_bridge::CvImage);
  msg_ptr_->image = im;
  cv::imshow("Image", msg_ptr_->image);   
  cv::waitKey(100);
  image_pub_.publish(msg_ptr_->toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_db_feed");
  
  ImageFeed image_feed(argv[1]);
    
  ros::spin();
  return 0;
}
