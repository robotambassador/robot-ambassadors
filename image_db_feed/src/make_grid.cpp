#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <string>
#include <fstream>
#include <iostream>

class MakeGrid
{
  void commandCallback(const std_msgs::Float32Ptr& msg);
  void publishNextImage();  
  int x_, y_, sizex_, sizey_;
  bool finished_;
  
  std::string format_;
  std::ofstream file;
  
  public:
  ros::NodeHandle nh_;
  ros::Publisher next_image_pub_;
  ros::Subscriber data_sub_;
  
  MakeGrid(int sizex = 9, int sizey = 16, std::string format = "ppm");
  
};

MakeGrid::MakeGrid(int sizex, int sizey, std::string format): sizex_(sizex), sizey_(sizey), format_(format)
{
  next_image_pub_ = nh_.advertise<std_msgs::String>("next_image", 1, true);
  data_sub_ = nh_.subscribe("rms_error", 1, &MakeGrid::commandCallback, this);
  
  finished_ = false; 
  x_ = y_ = 0;

  file.open("grid.html");
  file << "<table><tr>\n<td>";
  
  publishNextImage();
  
  while (!finished_ && nh_.ok())
  {
    ros::spinOnce();
  } 
}

void MakeGrid::commandCallback(const std_msgs::Float32Ptr& msg)
{
  file << msg->data << "</td>\n";
  x_++;
  
  if (x_ > sizex_)
  {
    x_=0;
    y_++;
    if (y_ > sizey_)
    {
      file << "</tr></table>";
      finished_ = true;
      ROS_INFO("Finished saving grid");
      return;
    }
    file << "</tr>\n<tr>\n<td>";
  }
  else
  {
    file << "<td>";
  }
  
  publishNextImage();
  
}

void MakeGrid::publishNextImage()
{
  std::stringstream sstm;
  sstm << x_ << "_" << y_ << "." << format_;
  std_msgs::String msg;
  msg.data = sstm.str();
  next_image_pub_.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "make_grid");
  
  MakeGrid make_grid;

  return 0;
}
