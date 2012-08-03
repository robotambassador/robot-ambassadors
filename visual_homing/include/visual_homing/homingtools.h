#ifndef HOMINGTOOLS
#define HOMINGTOOLS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

class HomingTools
{
  public:
  static float rms(cv::Mat& img1, cv::Mat& img2);
  static cv::Mat rotateImage(cv::Mat& img, float angle);
  static void turnAngle(bool clockwise, double radians, ros::NodeHandle& nh_, ros::Publisher& cmd_pub);
};

#endif
