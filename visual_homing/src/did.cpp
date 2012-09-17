#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <visual_homing/homingtools.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;


static const double MAX_ANGULAR = 3; // rad/s
static const double MAX_LINEAR = 0.5; // m/s

class DescentImageDistance
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber goal_image_sub_;
  
  ros::Publisher rms_pub_;
  ros::Publisher cmd_pub_;
  ros::ServiceServer home_srv_;
  ros::ServiceServer start_srv_;
  tf::TransformListener tf_listener_;
  
   //we will record transforms here for odometry
  tf::StampedTransform current_transform_;
  tf::StampedTransform start_transform_;
  tf::Vector3 desired_turn_axis_;
  double turn_radians_; 
  
  geometry_msgs::Twist next_cmd_;
  
  float last_rms_;
  bool homing_;
  bool turning_;
  tf::StampedTransform transform_;
  
  Mat home_;
  cv_bridge::CvImagePtr cv_image_;
  
  
  public:
  DescentImageDistance();  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void goalImageCallback(const sensor_msgs::ImageConstPtr& msg);
  bool saveHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool startHoming(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool checkTurn();
  void turnAngle(bool clockwise, double radians);
  bool execute();
};

DescentImageDistance::DescentImageDistance()
    : it_(nh_), homing_(false), turning_(false)
{
  goal_image_sub_ = it_.subscribe("goal_image", 1, &DescentImageDistance::goalImageCallback, this);
  image_sub_ = it_.subscribe("/gscam/image_raw", 1, &DescentImageDistance::imageCallback, this);
  
  rms_pub_ = nh_.advertise<std_msgs::Float32>("rms_error", 1, true);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  home_srv_ = nh_.advertiseService("save_home", &DescentImageDistance::saveHome, this);
  start_srv_ = nh_.advertiseService("start_homing", &DescentImageDistance::startHoming, this);
}

void DescentImageDistance::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (turning_) return;
  try
  {
    cv_image_ = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //wait for the listener to get the first message
    tf_listener_.waitForTransform("base_link", "odom",
                               ros::Time(0), ros::Duration(1.0));



    //record the starting transform from the odometry to the base frame
    tf_listener_.lookupTransform("base_link", "odom",
                              ros::Time(0), transform_);
  //cv::imshow("image", cv_image_->image);
  //cv::waitKey(5000);
  ROS_INFO("transform angle: %f, axis: %f", transform_.getRotation().getAngle(), transform_.getRotation().getAxis().getZ());
  cv::Mat im = HomingTools::rotateImage(cv_image_->image, transform_.getRotation().getAngle()*transform_.getRotation().getAxis().getZ());
  cv_image_->image = im;
  
  if (!home_.data) 
  {
    //ROS_WARNING("No home image found");
    return;
  }
  
  float rms_error = HomingTools::rms(cv_image_->image, home_);
  std_msgs::Float32 rms_msg;
  rms_msg.data = rms_error;
  
  rms_pub_.publish(rms_msg);
  ROS_DEBUG("Published rms as %f", rms_error);
  
  if (!homing_) return;
  if (!last_rms_) {
    last_rms_ = rms_error;
  }
  
  //geometry_msgs::Twist cmd;
  
  if (rms_error > last_rms_) {
    turnAngle(true, M_PI);
    return;
  } else {
    next_cmd_.angular.z = 0;
    next_cmd_.linear.x = -0.25;
  }  
  
  last_rms_ = rms_error;
  cmd_pub_.publish(next_cmd_);
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

bool DescentImageDistance::saveHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (cv_image_ == NULL) {
    ROS_ERROR("No image to save as home");
    return false;
  }
  home_ = cv_image_->image;
  return true;
}

bool DescentImageDistance::startHoming(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  homing_ = !homing_;
  return true;
}

// Checks if the turn command has been completed
bool DescentImageDistance::checkTurn()
{
  double angle_turned;
  if (turning_ && nh_.ok())
  {
    //get the current transform
    try
    {
      tf_listener_.lookupTransform("base_link", "odom", 
                                ros::Time(0), current_transform_);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      turning_ = false;
      return true;
    }
    tf::Transform relative_transform = 
      start_transform_.inverse() * current_transform_;
    tf::Vector3 actual_turn_axis = 
      relative_transform.getRotation().getAxis();
    angle_turned = relative_transform.getRotation().getAngle();
    if ( fabs(angle_turned) > 1.0e-2){
      if ( actual_turn_axis.dot( desired_turn_axis_ ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > turn_radians_) turning_ = false;
    }
  } else {
    turning_ = false;
  }
  if (!turning_) {
    ROS_INFO("angle turned = %f", angle_turned);
    next_cmd_.angular.z = 0.0;
    return true;
  }
  return false;
}

void DescentImageDistance::turnAngle(bool clockwise, double radians)
{
  turning_ = true;
  while(radians < 0) radians += 2*M_PI;
  while(radians > 2*M_PI) radians -= 2*M_PI;
  
  turn_radians_ = radians;

  //wait for the listener to get the first message
  tf_listener_.waitForTransform("base_link", "odom", 
                             ros::Time(0), ros::Duration(1.0));
  
  

  //record the starting transform from the odometry to the base frame
  tf_listener_.lookupTransform("base_link", "odom", 
                            ros::Time(0), start_transform_);
  
  //the command will be to turn at 0.25 rad/s
  next_cmd_.linear.x = 0.0;
  next_cmd_.angular.z = 0.25;
  
  if (clockwise) next_cmd_.angular.z = -next_cmd_.angular.z;
  
  //the axis we want to be rotating by
  
  desired_turn_axis_ = tf::Vector3(0,0,1);
  if (!clockwise) desired_turn_axis_ = -desired_turn_axis_;
}

bool DescentImageDistance::execute() {
  ros::Rate loop_rate(50);
  geometry_msgs::Twist prev_cmd;
  
  while (nh_.ok()) {
    //check if we have turned to desired angle
    if (turning_) checkTurn();
    
    
    //safety for bad maths
    if (next_cmd_.angular.z > MAX_ANGULAR)
    {
      next_cmd_.angular.z = MAX_ANGULAR;
      ROS_WARN("Max angular speed exceeded");
    }
    if (next_cmd_.linear.x > MAX_LINEAR)
    {
      next_cmd_.linear.x = MAX_LINEAR;
      ROS_WARN("Max linear speed exceeded");
    } 
    
    // avoid publishing the same message twice
    if ((next_cmd_.linear.x != prev_cmd.linear.x) || (next_cmd_.angular.z != prev_cmd.angular.z))
  	{
  	  cmd_pub_.publish(next_cmd_);
      prev_cmd = next_cmd_;
  	}
  	
  	ROS_DEBUG("published linear = %f,  angular = %f", next_cmd_.linear.x, next_cmd_.angular.z);
    
    ros::spinOnce(); 
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "did_homing");
  DescentImageDistance did;
  did.execute();
  return 0;
}
