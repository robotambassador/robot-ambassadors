/*
 * main.cpp
 *
 *  Created on: Mar 12, 2011
 *      Author: Nicu Stiurca
 *
 *  ROS node based on example on ROS wiki:
 *  http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 *
 *  Filtering/detection based on last year's IGVC lane detection
 *  I wrote.
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <sstream>

namespace enc = sensor_msgs::image_encodings;

static const char OCC[] = "occupancy";
static const char GRAY[] = "gray";
static const char SMOOTH[] = "smooth";
static const char ERODED[] = "Eroded";
static const char CANNY[] = "Canny";
static const char FLOOD[] = "Flooded";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher map_pub;
  size_t num_processed_;

  nav_msgs::OccupancyGrid grid;

  // parameters
  int smooth_amount_;
  double low_thresh_;
  double high_thresh_;
  int aperture_size_;

  bool disp_gray_;
  bool disp_smooth_;
  bool disp_eroded_;
  bool disp_canny_;
  bool disp_flood_;

  CvFont font;

public:
  ImageConverter() :
    it_(nh_), num_processed_(0),
    smooth_amount_(11),    //9
    low_thresh_(10.0),    //25
    high_thresh_(90.0),  //100
    aperture_size_(3),    //3

    disp_gray_(false),
    disp_smooth_(false),
    disp_eroded_(false),
    disp_canny_(true),
    disp_flood_(false)
  {
    ROS_DEBUG_STREAM("subscribing and advertising");

    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("image_raw", 1, &ImageConverter::imageCb, this);
    map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("map", 5);

    grid.data.resize(100);

    ROS_DEBUG_STREAM("creating named windows");

    if(disp_gray_)
      cv::namedWindow(GRAY);
    if(disp_smooth_)
      cv::namedWindow(SMOOTH);
    if(disp_eroded_)
      cv::namedWindow(ERODED);
    if(disp_canny_)
      cv::namedWindow(CANNY);
    if(disp_flood_)
      cv::namedWindow(FLOOD);

    ROS_DEBUG_STREAM("Constructed!");
  }

  ~ImageConverter()
  {
    if(disp_gray_)
      cv::destroyWindow(GRAY);
    if(disp_smooth_)
      cv::destroyWindow(SMOOTH);
    if(disp_eroded_)
      cv::destroyWindow(ERODED);
    if(disp_canny_)
      cv::destroyWindow(CANNY);
    if(disp_flood_)
      cv::destroyWindow(FLOOD);
  }

  void
  imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    const char* text;

    bool wait = false;
    cv_bridge::CvImagePtr in;
    cv_bridge::CvImage out;
    cv::Mat gray, warped, smooth, Canny, eroded, mask, occ;
    cv_bridge::CvImagePtr final;
    ROS_DEBUG_STREAM("Got a new frame to process");

    // first, convert the ROS data (in whatever format it may be in)
    // to an OpenCV-ready IplImage
    try
    {
      in = cv_bridge::toCvCopy(msg, enc::BGR8);
    } catch (cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("Failed to convert the ROS image to a IplImage: " << e.what());
      return;
    }

    cv::cvtColor(in->image, gray, CV_BGR2GRAY);

    if (disp_gray_)
    { // display the raw (grayscale) image?
      ROS_DEBUG_STREAM("Here's gray");
      cv::imshow(GRAY, gray);
      wait = true;
    }

    // convolve the image with a Gaussian kernel to smooth it
    // can be done in-place
    cv::GaussianBlur(gray, smooth, cv::Size(smooth_amount_, smooth_amount_), 0.0);
    if (disp_smooth_)
    { // display the warped image?
      ROS_DEBUG_STREAM("here's smooth");
      cv::imshow(SMOOTH, smooth);
      wait = true;h1c

    }

    cv::Mat element(5,5, CV_8UC1, 1);
    // erode the image to fill in gaps between lines
    cv::erode(smooth, eroded, element, cv::Point(-1, -1), 1);
    if (disp_eroded_)
    { // display the result of dilation?
      ROS_DEBUG_STREAM("here's eroded");
      cv::imshow(ERODED, eroded);
      wait = true;
    }

//    // dilate the image to fill in gaps between lines
//    cv::dilate(eroded, dilated, element, cv::Point(-1, -1), 1);
//    if (true /*this->m_bDispDilated*/)
//    { // display the result of dilation?
//      ROS_DEBUG_STREAM("here's dilated");
//      cv::imshow("Dilated", dilated);
//      wait = true;
//    }

    // run Canny edge detection on the (warped, smoothed) image
    cv::Canny(eroded, Canny, low_thresh_, high_thresh_, aperture_size_);
    if (disp_canny_)
    { // display the result of Canny edge detection?

	occ = in->image;

        cv::Rect gridSize(0, 0, Canny.cols/10, Canny.rows/10);
    	for(int col_shift = 0; col_shift < 10; col_shift++) {
            for(int row_shift = 0; row_shift < 10; row_shift++) {

		grid.info.map_load_time = ros::Time::now();
		grid.info.resolution = 1.5;
		grid.info.width = 15;
		grid.info.height = 10;

		geometry_msgs::Pose pose;
        	pose.position.x = 0;
        	pose.position.y = 0;
        	grid.info.origin = pose;

                gridSize.x = row_shift*gridSize.width;
                gridSize.y = col_shift*gridSize.height;

                //ROS_INFO("Top corner at (%i,%i).", gridSize.x, gridSize.y);
                //ROS_INFO("Opposite corner at (%i,%i).", gridSize.x + gridSize.width, gridSize.y + gridSize.height);

                cv::Mat gridChunk = Canny(gridSize);

                cv::Scalar occupied = cv::sum(gridChunk);
		
		std::stringstream ss;
                ss << occupied.val[0]/255.0;
                text = ss.str().c_str();
		
		cv::Scalar colour;

		int gridLoc = col_shift*10 + row_shift;
		//ROS_INFO("Filling occupanc grid location: %i", gridLoc);

		if (occupied.val[0]/255 > 20.0) {
			colour = cv::Scalar(0,0,255);
			grid.data[col_shift*10 + row_shift] = 100;
		} else {
			colour = cv::Scalar(0,255,0);
			grid.data[col_shift*10 + row_shift] = 0;
		}
		
		rectangle(occ, cv::Point(gridSize.x,gridSize.y), cv::Point(gridSize.x+gridSize.width,gridSize.y+gridSize.height), cv::Scalar(150,200,250));
                putText(occ, text, cv::Point(gridSize.x + gridSize.width/3, gridSize.y + gridSize.height/2), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, colour);

                //ROS_INFO("Occupied is %f", occupied.val[0]);
           }
      }

      ROS_DEBUG_STREAM("OMG HERE's CANNY");
      cv::imshow(CANNY, Canny);
      cv::imshow(OCC, occ);

      wait = true;
    }

    // flood-fill the lane
    cv::Point2i bottomCenter(Canny.cols/2, Canny.rows-5);
    cv::copyMakeBorder(Canny, mask, 1, 1, 1, 1, cv::BORDER_REPLICATE);
    cv::Scalar newVal(0.0, 255.0, 0.0);
    cv::Scalar diff(18, 18, 18, 0);
    out = *in;
    cv::floodFill(out.image, mask, bottomCenter, newVal, 0, diff, diff);
    if (disp_flood_)
    { // display the result of flood-filling the detected lane area?
      ROS_DEBUG_STREAM("Flood filled");
      cv::imshow(FLOOD, out.image);
      wait = true;
    }

    if (wait)
    {
      cvWaitKey(3);
    }

    image_pub_.publish(out.toImageMsg());
    map_pub.publish(grid);
  }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_detector");
  ImageConverter ic;
  ros::spin();
  return 0;
}

