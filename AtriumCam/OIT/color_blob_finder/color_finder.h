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


#ifndef _COLOR_FINDER_H_
#define _COLOR_FINDER_H_

#include "ros/ros.h"
#include "opencv/cv.h"
#include "oit_msgs/BlobArray.h"

class ColorFinder
{
  ros::Publisher blobs_pub;
  ros::Publisher world_pub;
  IplImage *backproject_img_, *mask_, *hue_;

  oit_msgs::BlobArray blobs_;

  // color histogram settings
  std::string color_histfile_;
  int smin_, vmin_, vmax_, hdims_;
  int min_area_;
  CvHistogram *hist_;
  CvMemStorage* storage_;
  IplConvKernel *kernel_;

  public:
    void init( std::string, std::string, int min_area = 5 );
    void image_cb( IplImage* img );
    void find_blobs(ros::Time t);

		int* smin() {return &smin_;}
		int* vmax() {return &vmax_;}
		int* vmin() {return &vmin_;}

    std::vector<oit_msgs::Blob> get_blobs() { return blobs_.blobs; }
    
};

#endif
