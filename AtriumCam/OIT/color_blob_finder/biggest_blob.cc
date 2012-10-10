/*
 * biggest_blob
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


#include <ros/ros.h>
#include <oit_msgs/BlobArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// need pinhole camera model
bool model_initialized = false;
double height = 0.0;//0.812;
double cam_height = 0.0;
std::string tfname;
image_geometry::PinholeCameraModel pcam;
tf::TransformBroadcaster* tb;
tf::TransformListener* listener;

void setHeightFromPublishedTransform()
{
  tf::StampedTransform height_tf, cam_height_tf;
  try
  {
    listener->lookupTransform("/ovh_height", "/ovh", ros::Time(0), cam_height_tf);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s",ex.what());
      return;
  }
  cam_height = cam_height_tf.getOrigin().z();
}


void camera_cb( const sensor_msgs::CameraInfoConstPtr& cam )
{
  if( !model_initialized )
  {
    pcam.fromCameraInfo( cam );
    model_initialized = true;
  }
}

void blob_cb( const oit_msgs::BlobArrayConstPtr& blobs )
{
  // for each blobarray message
  if( model_initialized )
  {
    setHeightFromPublishedTransform();
    int max_area = 0;
    int idx = -1;
    for( int i = 0; i < blobs->blobs.size(); i++ )
    {
      if( blobs->blobs[i].size > max_area )
      {
        idx = i;
        max_area = blobs->blobs[i].size;
      }
    }

    if( idx > -1 )
    {
      oit_msgs::Blob blob = blobs->blobs[idx];
      cv::Point2d uv;
      uv.x = blob.x + blob.width/2.0; uv.y = blob.y + blob.height/2.0;
      cv::Point2d uvrect;
      pcam.rectifyPoint( uv, uvrect );

      cv::Point3d imgray;
      pcam.projectPixelTo3dRay( uvrect, imgray );

      // project to correct distance from camera
      double fact = (cam_height-height)/imgray.z;
      double px = -fact*imgray.x; double py = fact*imgray.y;

      // publish transform
      tf::Quaternion quat; quat.setRPY(0,0,0);
      tf::Transform blobt( quat, tf::Point( -px, -py, -(cam_height-height) ));
      tb->sendTransform( tf::StampedTransform(blobt, blobs->header.stamp, "/cam_plane_flat", tfname ));
    }
  }
  else
  {
    ROS_WARN( "no camera model input, ignoring blob message" );
  }
}

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "blob_finder" );
  tb = new tf::TransformBroadcaster();
  listener = new tf::TransformListener();
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Subscriber blobs_sub = nh.subscribe( "child_blobs", 1, &blob_cb );
  ros::Subscriber camera_sub = nh.subscribe( "camera_info", 1, &camera_cb );
  nh_priv.param( "tfname", tfname, std::string("/child/base_link") );
  nh_priv.param( "height", height, 1.0 );
  ros::spin();
  return 0;
}

