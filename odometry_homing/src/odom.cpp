#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


class OdometryHoming
{
  ros::NodeHandle nh_;
  
  ros::Publisher cmd_pub_;
  //ros::Subscriber odom_sub_;
  ros::ServiceServer home_srv_;
  ros::ServiceServer start_srv_;
  tf::TransformListener tf_listener_;
  
  bool homing_;
  tf::StampedTransform home_;
  tf::StampedTransform position_;
  
  public:
  OdometryHoming();  
  void execute();
  bool saveHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool startHoming(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  //void odometryCallback(const nav_msgs::OdometryPtr& msg);
};

OdometryHoming::OdometryHoming()
{
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  home_srv_ = nh_.advertiseService("save_home", &OdometryHoming::saveHome, this);
  start_srv_ = nh_.advertiseService("start_homing", &OdometryHoming::startHoming, this);
  
  //odom_sub_ = nh_.subscribe("odom", &OdometryHoming::odometryCallback, 1);
  
  //wait for the listener to get the first message
  tf_listener_.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));
  
  homing_ = false;
}

bool OdometryHoming::saveHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  /*//record the starting transform from the odometry to the base frame
  tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), transform_);

  home_ = transform_.getOrigin();*/
  tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), home_);
  
  return true;
}

bool OdometryHoming::startHoming(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  
  homing_ = !homing_;
  return true;
}
/*
void OdometryHoming::odometryCallback(const nav_msgs::OdometryPtr& msg)
{
  position_ = msg.data;
  
  if(homing_)
  {

  }
}*/

void OdometryHoming::execute()
{
  ros::Rate loop_rate(10);
  while(nh_.ok())
  {
    tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), position_);
    
    tf::Transform diff = position_.inverse() * home_;
      double axis = diff.getRotation().getAxis().getZ();
      double angle = diff.getRotation().getAngle() * axis;
      
      ROS_INFO("angle: %f", angle);
    
    if (homing_)
    {
      geometry_msgs::Twist cmd;
      
  
      if (angle > 0.5*axis)
      {
        cmd.linear.x = 0;
        cmd.angular.z = 0.75;
      } else {
        cmd.linear.x = -0.4;
        cmd.angular.z = 0;
      }   
      
      cmd_pub_.publish(cmd);
               
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_homing");
  OdometryHoming odom;
  odom.execute();
  return 0;
}
