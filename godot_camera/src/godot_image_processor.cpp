#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
static const char OCC[] = "Occupancy";
static const char GRAY[] = "Gray";
static const char SMOOTH[] = "Smooth";
static const char ERODED[] = "Eroded";
static const char CANNY[] = "Canny";
static const char FLOODED[] = "Flooded";

class GodotImageprocessor
{
public:
  GodotImageprocessor();
  
private:
  void imageCallback(const sensor_msgs::Image::ConstPtr& raw_image);
  
  ros::NodeHandle nh_;

  ros::Publisher image_pub_;
  ros::Subscriber image_raw_sub_;
};


GodotImageprocessor::GodotImageprocessor()
{

  image_pub_ = nh_.advertise<sensor_msgs::Image>("/godot_image", 1);
	
  image_raw_sub_ = nh_.subscribe<sensor_msgs::Image>("/gscam/image_raw", 1, &GodotImageprocessor::imageCallback, this);

}

void GodotImageprocessor::imageCallback(const sensor_msgs::Image::ConstPtr& raw_image)
{
	ROS_DEBUG("Image Callback");
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat gray, warped, smooth, canny, eroded, mask, occ;
	
	int smooth_amount_ = 11;
  double low_thresh_ = 10.0;
  double high_thresh_ = 100.0;
  int aperture_size_ = 3;
	
  try
  {
    cv_ptr = cv_bridge::toCvCopy(raw_image, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  /*cv::imshow("Gray", gray);
  
  cv::GaussianBlur(gray, smooth, cv::Size(smooth_amount_, smooth_amount_), 0, 0);
  cv::imshow(SMOOTH, smooth);
 
  cv::Mat element(5,5,CV_8UC1, 1);
  cv::erode(smooth, eroded, element, cv::Point(-1,-1),1);
  cv::imshow(ERODED, eroded); */
  
  cv::Canny(gray, canny, low_thresh_, high_thresh_, aperture_size_);
  imshow(CANNY, canny);
  
   /* if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/

 // cv::imshow(WINDOW, cv_ptr->image);
  cv::waitKey(3);
    
  image_pub_.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "godot_image_processor");
  GodotImageprocessor godot_image_processor;

  ros::spin();
}
