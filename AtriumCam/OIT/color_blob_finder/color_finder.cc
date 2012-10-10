/*
 * color_finder
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
#include "color_finder.h"

#include "opencv/highgui.h"


void 
ColorFinder::image_cb( IplImage* hsv )
{
  if( hsv->width != mask_->width )
  {
    CvSize s = cvGetSize( hsv );
    hue_ = cvCreateImage( s, 8, 1 );
    mask_ = cvCreateImage( s, 8, 1 );
    backproject_img_ = cvCreateImage( s, 8, 1 );
  }

  cvInRangeS( hsv, cvScalar(0,smin_,MIN(vmin_,vmax_),0),
              cvScalar(180,256,MAX(vmax_,vmin_),0),mask_);
      
  cvSplit( hsv, hue_, 0, 0, 0 );
  cvCalcBackProject( &hue_, backproject_img_, hist_ );
  cvAnd( backproject_img_, mask_, backproject_img_, 0 );
  cvDilate( backproject_img_, backproject_img_, NULL, 2 );
  cvErode( backproject_img_, backproject_img_, NULL, 2 );
  //cvDilate( backproject_img_, backproject_img_, kernel_, 1 );

  //cvShowImage( color_histfile_.c_str(), backproject_img_ );
  //cvWaitKey(10);

}

void
ColorFinder::find_blobs( ros::Time t )
{
  CvSeq* c = 0;
  cvFindContours( backproject_img_, storage_, &c, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
  cvClearMemStorage( storage_ );

  blobs_.blobs.resize(0);

  // for each contour
  for( ; c != 0; c = c->h_next )
  {
    // if above size threshold
    double area = fabs( cvContourArea( c, CV_WHOLE_SEQ ));
    if( area > min_area_ )
    {
      //bounding box
      CvRect bb = cvBoundingRect( c, 1 );


      oit_msgs::Blob b;
      b.x = bb.x;
      b.y = bb.y;
      b.width = bb.width;
      b.height = bb.height;
      b.size = area;

      // add to blob list
      blobs_.blobs.push_back( b );
    }
  }
  blobs_.header.stamp = t;
  blobs_pub.publish( blobs_ );
}

void 
ColorFinder::init( std::string color_file, std::string name, int min_area )
  {

    color_histfile_ = color_file;
    min_area_ = min_area;
    printf( "filename: [%s]\n", color_histfile_.c_str() );

    // create histogram; read in hdims
    FILE* hist_dump = fopen( color_file.c_str(), "r" );
    fscanf( hist_dump, "%d\n", &hdims_ );
    fscanf( hist_dump, "%d\n", &smin_ );
    fscanf( hist_dump, "%d\n", &vmin_ );
    fscanf( hist_dump, "%d\n", &vmax_ );
    float* hranges = new float[2];
    hranges[0] = 0;
    hranges[1] = 180;
    hist_ = cvCreateHist( 1, &hdims_, CV_HIST_ARRAY, &hranges, 1 );
    float x = 0.0;
    for( int i = 0; i < hdims_; i++ )
    {
      fscanf( hist_dump, "%f\n", &x );
      cvSetReal1D( hist_->bins, i, x );
    }
    fclose( hist_dump );

    storage_ = cvCreateMemStorage(0);
    mask_ = cvCreateImage( cvSize(1,1),8,1);

    // read in kernel size
    int kernel_size = 5;

    kernel_ = cvCreateStructuringElementEx( kernel_size, kernel_size, 
                                            kernel_size/2, kernel_size/2, 
                                            CV_SHAPE_RECT );

    //cvNamedWindow( color_histfile_.c_str(), 1 );

    // declare publisher
    ros::NodeHandle n;
    std::string pub_name = name + "_blobs";
    std::string world_name = name + "_world";
    blobs_pub = n.advertise<oit_msgs::BlobArray>( pub_name, 1000 );


  }

