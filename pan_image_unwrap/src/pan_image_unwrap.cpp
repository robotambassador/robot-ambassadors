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

class ImageUnwrapper
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  int rotate_, magnitude_;
  
  public:
  ImageUnwrapper()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("gscam/image_raw", 1, &ImageUnwrapper::imageCallback, this);
    image_pub_ = it_.advertise("image_unwrapped", 1);
    cv::namedWindow("Unwrapped Image");
    nh_.param("rotate", rotate_, 90);
    nh_.param("magnitude", magnitude_, 80);
  }
  
  ~ImageUnwrapper() 
  {
    cv::destroyWindow("Image");
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("Received image");
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
    //imshow("original", cv_image->image);
    IplImage* dst = cvCreateImage( cvSize(src->width,src->height), 8, 3 );
    //IplImage* src2 = cvCreateImage( cvGetSize(src), 8, 3 );
    cvLogPolar( src, dst, cvPoint2D32f(src->width/2,src->height/2), magnitude_,
    CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
    //cvLogPolar( dst, src2, cvPoint2D32f(src->width/2,src->height/2), 80,
    //CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS+CV_WARP_INVERSE_MAP );
    Mat imgMat(dst);
    imgMat = rotateImage(imgMat, rotate_);
    //imshow("log-polar", imgMat);
    
    //ROS_INFO("src     height: %d, width: %d", src->height, src->width);
    //ROS_INFO("mat height: %d, width: %d", imgMat.rows, imgMat.cols);
    
    //Make a rectangle
    Rect roi((src->width - src->height)/2, src->height - 350, src->height, 100);
    //Point a cv::Mat header at it (no allocation is done)
    Mat image_roi = imgMat(roi);
      
    imshow("Unwrapped Image", image_roi);
    cv::waitKey(30);
    
    
    cv_bridge::CvImage out_msg;
    out_msg.header   = cv_image->header; // Same timestamp and tf frame as input image
    out_msg.encoding = cv_image->encoding; // Or whatever
    out_msg.image    = image_roi; // Your cv::Mat
    image_pub_.publish(out_msg.toImageMsg());
        
    //cvNamedWindow( "log-polar", 1 );
    //cvShowImage( "log-polar", dst );
    //cvNamedWindow( "inverse log-polar", 1 );
    //cvShowImage( "inverse log-polar", src2 );
  }  
  
  private:
  Mat rotateImage(const Mat& source, double angle)
  {
    Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    Mat dst;
    warpAffine(source, dst, rot_mat, source.size());
    return dst;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pan_image_unwrapper");
  ImageUnwrapper iu;
  ros::spin();
  return 0;
}
