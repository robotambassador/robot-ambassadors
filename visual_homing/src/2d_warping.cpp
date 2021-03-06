#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc_c.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

static const char WINDOW[] = "Warped Image";

class 2D_Warping
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  int rotate_, magnitude_;
  
  public:
  2D_Warping()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("image_unwrapped", 1, &2D_Warping::imageCallback, this);
    image_pub_ = it_.advertise("image_unwrapped", 1);
    cv::namedWindow(WINDOW);
    //nh_.param("rotate", rotate_, 90);
  }
  
  ~2D_Warping() 
  {
    cv::destroyWindow(WINDOW);
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};

void 2D_Warping::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  IplImage img = cv_image->image;
  IplImage* src = &img;
  
  //Mat imgMat(dst);
  imgMat = rotateImage(imgMat, rotate_);
  //imshow("log-polar", imgMat);

  cv::waitKey(3);
  
  cv_bridge::CvImage out_msg;
  out_msg.header   = cv_image->header; // Same timestamp and tf frame as input image
  out_msg.encoding = cv_image->encoding; // Or whatever
  out_msg.image    = image_roi; // Your cv::Mat
  image_pub_.publish(out_msg.toImageMsg());
}  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "2d_warping");
  ImageUnwrapper iu;
  ros::spin();
  return 0;
}
