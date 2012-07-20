#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <string>

class CommandListener
{
  void commandCallback(const geometry_msgs::Twist& cmd);
  void publishNextImage();  
  int x_, y_;
  double angle_;
  
  std::string format_;
  
  public:
  ros::NodeHandle nh_;
  ros::Publisher next_image_pub_;
  ros::Subscriber cmd_sub_;
  
  CommandListener(int x = 0, int y = 0, double angle = 0, std::string format = "ppm");
  
};

CommandListener::CommandListener(int x, int y, double angle, std::string format): x_(x), y_(y), angle_(angle), format_(format)
{
  next_image_pub_ = nh_.advertise<std_msgs::String>("next_image", 1, true);
  cmd_sub_ = nh_.subscribe("cmd_vel", 1, &CommandListener::commandCallback, this); 

  publishNextImage();
}

void CommandListener::commandCallback(const geometry_msgs::Twist& cmd)
{
  if (cmd.linear.x > 0) {
    if (angle_ > -0.785398163 && angle_ < 0.785398163)
      x_+=1;
    else if (angle_ > 2.35619449 || angle_ < -2.35619449)
      x_-=1;
    else if (angle_ > 0.785398163 && angle_ < 2.35619449)
      y_+=1;
    else if (angle_ > -2.35619449 && angle_ < -0.785398163)
      y_-=1;
  }
  
  angle_ += cmd.angular.z;
  if (angle_ > 3.14)
    angle_ -= 6.28;
  else if (angle_ <= -3.14)
    angle_ += 6.28;
  
  publishNextImage();
  
  ROS_INFO("received cmd, new pos x: %d, y: %d, angle: %f", x_, y_, angle_);
}

void CommandListener::publishNextImage()
{
  std::stringstream sstm;
  sstm << x_ << "_" << y_ << "." << format_;
  std_msgs::String msg;
  msg.data = sstm.str();
  next_image_pub_.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_listener");
  
  CommandListener command_listener;
    
  ros::spin();
  return 0;
}
