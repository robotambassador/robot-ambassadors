/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010. David Feil-Seifer
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxtypes.h>
#include <image_transport/image_transport.h>

image_transport::CameraSubscriber sub_;
image_transport::CameraPublisher pub_;
sensor_msgs::CvBridge img_bridge_;

int border;

void image_cb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg )
{
  if (msg->encoding.find("bayer") != std::string::npos)
    boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";

  if (img_bridge_.fromImage(*msg, "bgr8"))
  {
    IplImage *orig, *dest;
    orig = img_bridge_.toIpl();

    // make new opencv image with border pixels around image */

		CvPoint offset; offset.x = border; offset.y = border;
		dest = cvCreateImage(cvSize(orig->width+border*2,orig->height+border*2), 8, 3 );
		cvZero(dest);
    cvCopyMakeBorder(orig,dest,offset,IPL_BORDER_CONSTANT,cvScalar(0,0,0));

    // convert Ipl to sensor_msgs::Image
		sensor_msgs::ImagePtr newimg = sensor_msgs::CvBridge::cvToImgMsg(dest, "bgr8");
    newimg->header.frame_id = msg->header.frame_id;

    // now change camera_info so that cx,cy is moved border,border pixels
    sensor_msgs::CameraInfo newmsg;
    newmsg = *info_msg;

    newmsg.height += 2*border;
  	newmsg.width += 2*border;

    double fx = newmsg.P[0] ;
    double fy = newmsg.P[5] ;

    double cx = newmsg.P[2] + border;
    double cy = newmsg.P[6] + border;

	  newmsg.P[2] = cx;
  	newmsg.P[6] = cy;
    newmsg.K[2] = cx;
    newmsg.K[5] = cy;

	  newmsg.P[0] = fx;
	  newmsg.P[5] = fy;


    // publish letterboxed data
		pub_.publish( *newimg, newmsg, info_msg->header.stamp );

    // clean up
    cvReleaseImage(&dest);
  }
  else
    ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
}

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "image_letterboxer" );
  ros::NodeHandle nh;
  nh.param("border", border, 100 );
  image_transport::ImageTransport it(nh);
  sub_ = it.subscribeCamera("image_raw", 1, &image_cb);
  pub_ = it.advertiseCamera("letterbox/image_raw", 1);
  ros::spin();

  return 0;
}
