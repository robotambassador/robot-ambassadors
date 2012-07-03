/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
//#include <cv_bridge/CvBridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using message_filters::Synchronizer;
using message_filters::sync_policies::ExactTime;
using message_filters::sync_policies::ApproximateTime;
#include <pcl/point_types.h>

using boost::make_shared;

namespace flyer_kinect
{

using std::string;
using std::vector;
using std::swap;
using std::max;
using std::min;

const int roi_startx = 12;
const int roi_starty = 72;
const int roi_width = 140;
const int roi_height = 48;

class KinectLKDemo : public nodelet::Nodelet
{
public:
  typedef sensor_msgs::PointCloud2 PointCloud2;
private:
  ros::NodeHandle nh_priv;
  ros::NodeHandle nh;
  // Params
  bool debug_window;
  bool bail_early;
  // Publishers
  ros::Publisher marker_pub;
  image_transport::Publisher image_pub;
  // Message Filter stuff
  boost::shared_ptr<Synchronizer<ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Image> > > sync_cloud_image;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_filter;
  message_filters::Subscriber<sensor_msgs::Image> sub_image_filter;
  // Subscribers
  //ros::Subscriber chatter_sub;
  image_transport::Subscriber image_sub;
  // Timers
  ros::Timer output_timer;
  // Image Transports
  image_transport::ImageTransport img_txp;
  // Members
  cv::Mat prevFrame;
  bool need_to_init;
  vector<cv::Point2f> points[2];
  vector<cv::Point3f> points_3d[2];
  vector<uchar> prev_status;
  int max_queue_size;
  boost::mutex mutex;

public:
  KinectLKDemo() :
    debug_window(false), bail_early(false), img_txp(nh), need_to_init(true), max_queue_size(5)
  {
  }
private:
  void onInit()
  {
    nh = getMTNodeHandle();
    nh_priv = getMTPrivateNodeHandle();

    img_txp = image_transport::ImageTransport(nh);

    NODELET_INFO("onInit()");

    if (debug_window)
    {
      //      cv::namedWindow("test", 0);
      //      cv::namedWindow("test2", 0);
      cv::namedWindow("corners", 0);
    }

    // Parameters
    nh_priv.param("debug_window", debug_window, debug_window);
    nh_priv.param("bail_early", bail_early, bail_early);
    //    nh_priv.param("awesome_factor", awesome_factor, awesome_factor);
    // Publishers
    //awesome_pub = nh_priv.advertise<std_msgs::String> ("awesome", 10);
    image_pub = img_txp.advertise("output", 10, true);
    //info_pub = nh.advertise<std_msgs::String> ("info", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker> ("marker", 10);
    // Subscribers
    //    image_sub = img_txp.subscribe("input", 1, &KinectLKDemo::imageCb, this);
    sub_cloud_filter.subscribe(nh_priv, "cloud", max_queue_size);
    sub_image_filter.subscribe(nh_priv, "image", max_queue_size);
    sync_cloud_image
        = make_shared<Synchronizer<ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Image> > > (max_queue_size);
    sync_cloud_image->connectInput(sub_cloud_filter, sub_image_filter);
    sync_cloud_image->registerCallback(bind(&KinectLKDemo::cloudImageCb, this, _1, _2));

    // Timers
    //    output_timer = nh.createTimer(ros::Duration(0.1), &MyNodelet::outputTimerCb, this);
  }

  //  void outputTimerCb(const ros::TimerEvent& e)
  //  {
  //    std_msgs::String output;
  //    std::ostringstream s;
  //    s << "foo bar baz " << awesome_factor;
  //    output.data = s.str();
  //    awesome_pub.publish(output);
  //  }

  //  void chatterCb(const std_msgs::StringConstPtr& chatter)
  //  {
  //    NODELET_INFO_STREAM("Heard chatter: " << chatter->data);
  //  }

  void cloudImageCb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    boost::mutex::scoped_lock lock(mutex);
    NODELET_INFO_STREAM("Got cloud with " << (cloud_msg->width * cloud_msg->height) << " points and image, "
        << msg_ptr->width << "x" << msg_ptr->height << ", enc: " << msg_ptr->encoding << " data.size()="
        << msg_ptr->data.size());
    cv::Mat cv_img;
    // The following takes the input, which is a 640*480-element array of unsigned ints
    // arranged in GRBG Bayer encoding, and turns it into a 160*120-element array of
    // 4-channel (GRGR) pixels.
    // Input:
    //  G R G R  G R G R  G R G R  G R G R
    //  B G B G  B G B G  B G B G  B G B G
    //  G R G R  G R G R  G R G R  G R G R
    //  B G B G  B G B G  B G B G  B G B G
    //
    //  G R G R  G R G R  G R G R  G R G R
    //  B G B G  B G B G  B G B G  B G B G
    //  G R G R  G R G R  G R G R  G R G R
    //  B G B G  B G B G  B G B G  B G B G
    //
    //  G R G R  G R G R  G R G R  G R G R
    //  B G B G  B G B G  B G B G  B G B G
    //  G R G R  G R G R  G R G R  G R G R
    //  B G B G  B G B G  B G B G  B G B G
    //
    //  Output:
    //  G G G G
    //  G G G G
    //  G G G G
    //  (where these G's are the first (upper-left) one in each block of 4x4 pixels in the original image)
    // thus the output array has 19,200 unsigned ints, i.e. 16x smaller than the input
    // Step 1: create a 160x120, 4-channel image -- this involves no copying!
    // here, 3 out of every 4 rows are effectively thrown away with the step=640*4=2560 argument
    cv_img = cv::Mat(120, 160, CV_8UC4, (void*)(&(msg_ptr->data[0])), 2560);
    // Reduce further by taking a rectangular ROI of mostly the bottom of the image:
    cv::Mat img_roi(cv_img, cv::Rect(roi_startx, roi_starty, roi_width, roi_height));
    //cv::imshow("test", cv_img);

    // Step 2: extract just "channel 1", i.e. the upper-left G pixel from each original 4x4 block:
    cv::Mat green(img_roi.rows, img_roi.cols, CV_8UC1);
    cv::Mat out[] = {green};
    int from_to[] = {0, 0};
    cv::mixChannels(&img_roi, 1, out, 1, from_to, 1);
    //cv::imshow("test2", green);

    if (bail_early)
      return;

    const cv::Size winSize(10, 10);
    const cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01);

    cv::Mat color;

    if (debug_window)
    {
      // Make a color image for display:
      cv::cvtColor(green, color, CV_GRAY2RGB);
    }

    need_to_init = (need_to_init or points[1].size() < 5);
    vector<uchar> status;
    vector<int> matchidx;
    if (need_to_init)
    {
      // Now, pick out 'strong corners':
      const int MAX_COUNT = 50;
      const double qualityLevel = 0.01; //Characterizes the minimal accepted quality of image corners; the value of the parameter is multiplied by the by the best corner quality measure (which is the min eigenvalue, see cornerMinEigenVal() , or the Harris function response, see cornerHarris() ). The corners, which quality measure is less than the product, will be rejected. For example, if the best corner has the quality measure = 1500, and the qualityLevel=0.01 , then all the corners which quality measure is less than 15 will be rejected.
      const double minDistance = 10; // The minimum possible Euclidean distance between the returned corners
      const cv::Mat mask = cv::Mat(); // The optional region of interest. If the image is not empty (then it needs to have the type CV_8UC1 and the same size as image ), it will specify the region in which the corners are detected
      const int blockSize = 3; // Size of the averaging block for computing derivative covariation matrix over each pixel neighborhood, see cornerEigenValsAndVecs()
      const bool useHarrisDetector = false; // useHarrisDetector – Indicates, whether to use operator or cornerMinEigenVal()
      const double k = 0.04; // k – Free parameter of Harris detector
      vector<cv::Point2f> new_points;
      int max_count = 50; // - points[1].size();
      cv::goodFeaturesToTrack(green, new_points, MAX_COUNT, qualityLevel, minDistance, mask, blockSize,
                              useHarrisDetector, k);
      NODELET_DEBUG("goodFeaturesToTrack returned %d points", (int)new_points.size());
      // refine to subpixel accuracy (necessary?):
      if (new_points.size() > 0)
      {
        cv::cornerSubPix(green, new_points, winSize, cv::Size(-1, -1), termcrit);
      }
      //points[1].insert(points[1].end(), new_points.begin(), new_points.end()); //append new points
      points[1] = new_points;
    }
    else if (not points[0].empty())
    {
      vector<float> err;
      float err_max = 0;
      float err_min = 9999;
      if (prevFrame.empty())
        green.copyTo(prevFrame);
      calcOpticalFlowPyrLK(prevFrame, green, points[0], points[1], status, err, winSize, 3, termcrit, 0.5, 0);
      size_t i, k;
      for (i = k = 0; i < points[1].size(); i++)
      {
        if (status[i]) // point was tracked
        {
          err_max = max(err_max, err[i]);
          err_min = min(err_min, err[i]);
          points[1][k++] = points[1][i];
          matchidx.push_back(i);
          //points[0][k++] = points[0][i];
          if (debug_window)
          {
            if ((not prev_status.empty()) and not (need_to_init))
            {
              cv::line(color, points[0][i], points[1][i], cv::Scalar(255, 0, 0), 1);
            }
            cv::circle(color, points[1][i], 1, cv::Scalar(0, (int)min(255 * 5 / err[i], (float)255.0), 0), -1);
          }
        }
      }
      //points[0].resize(k);
      points[1].resize(k);
      NODELET_INFO("err_min = %f, err_max = %f", err_min, err_max);
      points2DTo3D(cloud_msg);
      publishPointMarkers(not need_to_init, matchidx);
    }
    need_to_init = false;

    if (debug_window)
    {
      cv::imshow("corners", color);
      cv::waitKey(3);
    }
    NODELET_INFO("Tracking %d points", (int)points[1].size());

    //    sensor_msgs::Image output_msg;
    //    output_msg.encoding = "8UC1";
    //    output_msg.height = 240;
    //    output_msg.width = 640;
    //    output_msg.step = 1280;
    //    output_msg.data.resize(240*1280);
    //    memcpy((void *)(&(output_msg.data[0])), cv_img.data, 240*1280);
    //    image_pub.publish(output_msg);

    points[0] = points[1];
    points_3d[0] = points_3d[1];
    prev_status = status;
    swap(prevFrame, green);

  }

  void points2DTo3D(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    points_3d[1].resize(points[1].size());
    for (int i = 0; i < points[1].size(); i++)
    {
      // convert tracked point coordinates back to 640x480 space:
      int pt_x = floor(4 * (points[1][i].x + roi_startx) + 0.5);
      int pt_y = floor(4 * (points[1][i].y + roi_starty) + 0.5);
      NODELET_INFO("Looking for point at x=%d, y=%d", pt_x, pt_y);
      NODELET_INFO_STREAM("row step = " << (cloud->point_step * 640) << "; cloud->point_step = " << cloud->point_step);
      //      pcl::PointXYZ point_3d;
      float* p;
      p = (float *)&(cloud->data[pt_y * cloud->point_step * 640 + pt_x * cloud->point_step]);
      //      point_3d.x = p[0];
      //      point_3d.y = p[1];
      //      point_3d.z = p[2];
      //if (not (isnan(p[0]) or isnan(p[1]) or isnan(p[2])))
      {
        points_3d[1].push_back(cv::Point3f(p[0], p[1], p[2]));
        NODELET_INFO("Point location = %f %f %f", p[0], p[1], p[2]);
      }
    }
  }

  void publishPointMarkers(const bool draw_lines, const vector<int>& match_idx)
  {
    if (marker_pub.getNumSubscribers())
    {
      if (not (points_3d[0].empty() or points_3d[1].empty()))
      {
        visualization_msgs::Marker points;
        visualization_msgs::Marker lines;
        lines.header.frame_id = points.header.frame_id = "kinect_depth";
        lines.header.stamp = points.header.stamp = ros::Time::now();
        points.ns = "points";
        lines.ns = "lines";
        lines.action = points.action = visualization_msgs::Marker::MODIFY;
        //      points.pose.position.x = points_location.x;
        //      points.pose.position.y = points_location.y;
        //      points.pose.position.z = points_location.z;
        lines.pose.orientation.w = points.pose.orientation.w = 1.0;
        lines.id = points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;
        lines.type = visualization_msgs::Marker::LINE_LIST;
        lines.scale.x = 0.01;
        lines.scale.y = 0.01;
        lines.scale.z = 0.01;
        points.scale.x = 0.03;
        points.scale.y = 0.03;
        points.scale.z = 0.03;
        points.color.r = 0.0;
        points.color.g = 1.0;
        points.color.b = 0.0;
        points.color.a = 0.8;
        lines.color.r = 0.0;
        lines.color.g = 0.0;
        lines.color.b = 1.0;
        lines.color.a = 0.8;
        lines.lifetime = points.lifetime = ros::Duration(0);
        for (int i = 0; i < points_3d[1].size(); i++)
        {
          if (prev_status[i] and (not (isnan(points_3d[0][i].x) or isnan(points_3d[0][i].y) or isnan(points_3d[0][i].z)
              or isnan(points_3d[1][i].x) or isnan(points_3d[1][i].y) or isnan(points_3d[1][i].z))))
          {
            geometry_msgs::Point p0, p1;
            p0.x = points_3d[0][match_idx[i]].x;
            p0.y = points_3d[0][match_idx[i]].y;
            p0.z = points_3d[0][match_idx[i]].z;
            p1.x = points_3d[1][i].x;
            p1.y = points_3d[1][i].y;
            p1.z = points_3d[1][i].z;
            points.points.push_back(p1);
            if (draw_lines)
            {
              lines.points.push_back(p0);
              lines.points.push_back(p1);
            }
          }
        }

        marker_pub.publish(points);
        marker_pub.publish(lines);
      }
    }
  }

};
PLUGINLIB_DECLARE_CLASS(flyer_kinect, KinectLKDemo, flyer_kinect::KinectLKDemo, nodelet::Nodelet)
;
}
