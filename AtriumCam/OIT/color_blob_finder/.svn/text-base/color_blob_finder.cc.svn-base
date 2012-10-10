/*
 * color_blob_finder
 * Copyright (c) 2010, David Feil-Seifer
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "ros/ros.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "image_transport/image_transport.h"
//#include "image_geometry/pinhole_camera_model.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/fill_image.h"

//#include "contour_fcns.h"
#include "color_finder.h"

//#include "ray.h"

using namespace std;

class ImageSplitter
{
public:
  ros::NodeHandle n;
private:
  ros::Publisher hsv_pub;
  ros::Publisher foreground_pub;

  sensor_msgs::CvBridge img_bridge_;
  sensor_msgs::Image img_;
  IplImage *hsv_img_, *disp_;
	bool first;
  int display;

  IplConvKernel *kernel_;
  CvMemStorage* storage_;
  vector<ColorFinder> color_finders_;
  vector<CvScalar> colors_;

	//int vmin, vmax, smin;

  public:

  void image_cb( const sensor_msgs::ImageConstPtr& msg )
  {
    sensor_msgs::Image img_msg = *(msg.get());
    if( img_bridge_.fromImage( img_msg, "bgr8" ) )
    {
      ros::Time img_time = msg->header.stamp;
      IplImage* frame = img_bridge_.toIpl();

      if( frame->width != hsv_img_->width )
      {
        ROS_WARN( "resizing images to: [%d,%d]", cvGetSize(frame).width, cvGetSize(frame).height );
        //resize images
        cvReleaseImage( &hsv_img_ );
        cvReleaseImage( &disp_ );
        hsv_img_ = cvCreateImage( cvGetSize(frame), 8, 3 );
        disp_ = cvCreateImage( cvGetSize(frame), 8, 3 );
      }
      cvCopy( frame, disp_ );
      cvCvtColor( frame, hsv_img_, CV_BGR2HSV );
      
      // color finders
      int c = 0;
      for( vector<ColorFinder>::iterator i = color_finders_.begin();
            i != color_finders_.end(); i++ )
      {
        // color probability images
        i->image_cb( hsv_img_ );
        // find contours
        i->find_blobs(img_msg.header.stamp);
        // publish blobs

        // render blobs
        vector<oit_msgs::Blob> blobs = i->get_blobs();
        for( unsigned int j = 0; j < blobs.size(); j++ )
        {
          oit_msgs::Blob b = blobs[j];
          cvRectangle( disp_, cvPoint( b.x, b.y ), 
                              cvPoint( b.x+b.width, b.y+b.height ),
                              colors_[c], 1 );         
        }
        c++;
      }

      // robot detector

			/*
        // contours

      cvErode( foreground_thresh_, foreground_thresh_, NULL, 2 );
      cvDilate( foreground_thresh_, foreground_thresh_, NULL, 2 );
      //cvDilate( foreground_thresh_, foreground_thresh_, kernel_, 1 );
      cvCvtScale( foreground_thresh_, foreground_mask_, 1, 0 );
      cvZero( foreground_ );
      cvCopy( frame, foreground_, foreground_mask_ );
      //cvShowImage( "foreground", foreground_ );

      CvSeq* contour = 0;
      cvFindContours( foreground_mask_, storage_, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
      cvClearMemStorage( storage_ );

      CvRect bb;
      oit_msgs::BlobArray blobs;
      for( ; contour != 0; contour = contour->h_next )
      {

        cvDrawContours(disp_,contour,CV_RGB(0,255,0), CV_RGB(255,0,0), 0, 1, CV_AA, cvPoint(0,0));
        bb = cvBoundingRect(contour, 1 );
        oit_msgs::Blob b;
        b.x = bb.x;
        b.y = bb.y;
        b.width = bb.width;
        b.height = bb.height;
        cvRectangle( disp_, cvPoint( b.x, b.y ),
                            cvPoint( b.x+b.width, b.y+b.height ),
                            CV_RGB(255,0,0), 1 );
        blobs.blobs.push_back(b);
      }

      foreground_pub.publish( blobs );
        // undo robot contour
        // undo child contour
      */

      // TODO: fill for foreground color
      if( display > 0 )
      {
        cvShowImage( "output", disp_ );
        cvWaitKey(10);
      }
    }
    else
    {
      ROS_WARN( "img_bridge could not parse image" );
    }
  }

  void init()
  {
    //hsv_pub = n.advertise<sensor_msgs::Image>("image_hsv",1000);
    foreground_pub = n.advertise<oit_msgs::BlobArray>("foreground_blobs",1000);
    n = ros::NodeHandle("~");
    std::string colorfile, irfile, childfile;
    n.param("parent_hist", colorfile, std::string(""));
    n.param("child_hist", childfile, std::string(""));
    n.param("ir_hist", irfile, std::string(""));

    hsv_img_ = cvCreateImage( cvSize( 320,240), 8, 3 );
    disp_ = cvCreateImage( cvSize( 320,240), 8, 3 );
    //foreground_img_ = cvCreateImage( cvSize( 320,240), 8, 1 );

    // colors
    color_finders_.clear();
    
    if( irfile != std::string("") )
    {
      ColorFinder c;
      c.init( irfile.c_str(), "ir" );
      color_finders_.push_back( c );
      colors_.push_back( CV_RGB( 0, 0, 255 ) );
    }

		if( childfile != std::string( "" ) )
    {
      ColorFinder p;
      p.init( childfile.c_str(), "child" );
      color_finders_.push_back( p );
      colors_.push_back( CV_RGB( 255, 128, 0 ) );
    }

    if( colorfile != std::string( "" ) )
    {
      ColorFinder p;
      p.init( colorfile.c_str(), "parent" );
      color_finders_.push_back( p );
      colors_.push_back( CV_RGB( 128, 255, 0 ) );
    }
    //std::string background_file;
    //n.param( "background", background_file, std::string("") );
    n.param( "display", display, 1 );
    if( display > 0 )
    {
      cvNamedWindow( "output", 1 );
			cvCreateTrackbar( "Vmin", "output", color_finders_[1].vmin(), 256, 0);
			cvCreateTrackbar( "Vmax", "output", color_finders_[1].vmax(), 256, 0);
			cvCreateTrackbar( "Smin", "output", color_finders_[1].smin(), 256, 0);
    }

    kernel_ = cvCreateStructuringElementEx( 15, 15, 8, 8, CV_SHAPE_RECT );
    storage_ = cvCreateMemStorage(0);

		first = true;
  }

  void cleanup()
  {
  }
};

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"color_blob_finder" );
  ImageSplitter* i = new ImageSplitter();
  boost::shared_ptr<ImageSplitter> foo_object(i);
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("image_raw", 1, &ImageSplitter::image_cb, foo_object );
  i->init();
  ros::spin();
  ROS_INFO( "image_splitter quitting..." );
  i->cleanup();
  return 0;
}
