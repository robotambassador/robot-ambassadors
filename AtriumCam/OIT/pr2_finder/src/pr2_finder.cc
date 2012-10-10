/*
 * ir_finder
 * Copyright (c) 2010, David Feil-Seifer and Edward Kaszubski
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
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "LinearMath/btMatrix3x3.h"
#include <math.h>

#include <nav_msgs/Odometry.h>

// need pinhole camera model
bool model_initialized = false;
double height = 0.0;//0.812;
double cam_height = 0.0;
double blob_dist_baseline;
double blob_dist_tolerance;
image_geometry::PinholeCameraModel pcam;
sensor_msgs::CvBridge img_bridge_;
tf::TransformBroadcaster* tb;
tf::TransformListener* listener;

IplImage* frame;
oit_msgs::BlobArrayConstPtr reds_, yellows_;

void setHeightFromPublishedTransform()
{
  tf::StampedTransform height_tf, cam_height_tf;
  try
  {
    listener->lookupTransform("/ovh_height", "/ovh", ros::Time(0), cam_height_tf);
    listener->lookupTransform("/pr2_ovh_target", "/base_link", ros::Time(0), height_tf);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s",ex.what());
      return;
  }
  cam_height = cam_height_tf.getOrigin().z();
	//height = 1.25;
  height = -height_tf.getOrigin().z();
}

void image_cb( const sensor_msgs::ImageConstPtr &img )
{
	sensor_msgs::Image img_msg = *(img.get());
	if( img_bridge_.fromImage( img_msg, "bgr8" ) )
	{
		frame = img_bridge_.toIpl();
	}

}


void camera_cb( const sensor_msgs::CameraInfoConstPtr& cam )
{
  //if( !model_initialized )
  {
    pcam.fromCameraInfo( cam );
    model_initialized = true;
  }
}

void red_cb(const oit_msgs::BlobArrayConstPtr& reds )
{
	reds_ = reds;
}

void yellow_cb(const oit_msgs::BlobArrayConstPtr& yellows )
{
	yellows_ = yellows;
}


int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "pr2_finder" );
  tb = new tf::TransformBroadcaster();
  listener = new tf::TransformListener();
  ros::NodeHandle nh;
  nh.param("blob_dist_baseline", blob_dist_baseline, 0.3);
  nh.param("blob_dist_tolerance", blob_dist_tolerance, 0.25);
  ros::Subscriber yellow_sub = nh.subscribe( "blue_blobs", 1, &yellow_cb );
  ros::Subscriber red_sub = nh.subscribe( "red_blobs", 1, &red_cb );
  ros::Subscriber camera_sub = nh.subscribe( "camera_info", 1, &camera_cb );

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("ovh_odom",100);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, image_cb );

	ros::Rate loop_rate(30);

	ros::Time last_time = ros::Time::now();

	cvNamedWindow("pr2_finder",0);

	// construct the base odometry message
	nav_msgs::Odometry odom;
	odom.header.frame_id = "ovh";
	odom.pose.covariance[0] = 60;
	odom.pose.covariance[7] = 60;
	odom.pose.covariance[14] = 60;
	odom.pose.covariance[21] = 999999;
	odom.pose.covariance[28] = 999999;
	odom.pose.covariance[35] = 2380;

	odom.pose.pose.position.z = 0;

	while( ros::ok() )
	{
		loop_rate.sleep();
		ros::spinOnce();
		if( !yellows_ || !reds_ ) continue;
		// find biggest yellow blob
		if( yellows_->header.stamp > last_time && (reds_->header.stamp - yellows_->header.stamp).toSec() < 0.03 )
		{
			last_time = yellows_->header.stamp;
			// if new data
			
			int max_area = 0;
			oit_msgs::Blob yb;

			for( int i = 0; i < yellows_->blobs.size(); i++ )
			{
				if( yellows_->blobs[i].size > max_area )
				{
					max_area = yellows_->blobs[i].size;
					yb = yellows_->blobs[i];
				}
			}

			double min_dist = 999;
			oit_msgs::Blob rb;

			// find nearby red dot
			for( int i = 0; i < reds_->blobs.size(); i++ )
			{
				oit_msgs::Blob b = reds_->blobs[i];
				double d = hypot( (b.x+b.width/2)-(yb.x-yb.width/2), (b.y+b.height/2)-(yb.y+yb.height/2) );
				if( d < min_dist )
				{
					min_dist = d;
					rb = b;
				}
			}

			cvCircle( frame, cvPoint( yb.x+yb.width/2, yb.y+yb.height/2 ), 5, CV_RGB(0,0,255), -1 );
			cvCircle( frame, cvPoint( rb.x+rb.width/2, rb.y+rb.height/2 ), 3, CV_RGB(0,255,0), -1 );

			cvShowImage("pr2_finder",frame );
			cvWaitKey(10);
	

		  if( model_initialized )
  		{
				//ROS_INFO( "blobs.size: %d", blobs->blobs.size() );

	    	setHeightFromPublishedTransform();
	  	  //ROS_INFO( "height: %0.2f cam_height: %0.2f", height, cam_height );
  	  	// rectify blobs to real-world points
    
	  	  tf::StampedTransform floor_to_cam, ray1_tf, ray2_tf;
		
				try
				{
					listener->waitForTransform("/ovh", "/ovh_height", ros::Time(0), ros::Duration(0.250));
					listener->lookupTransform("/ovh", "/ovh_height", ros::Time(0), floor_to_cam);
				}
				catch( tf::TransformException &ex )
				{
					ROS_WARN( "unable to do transformation: [%s]", ex.what());
					continue;
				}
	
				double ir_plane = height - floor_to_cam.getOrigin().z();
	
				tf::Transform floor_to_cam_pub ( tf::Quaternion( 0, 0, 0), floor_to_cam.getOrigin() );
	
				tb->sendTransform( tf::StampedTransform(floor_to_cam_pub, yellows_->header.stamp, "/ovh", "/cam_plane_flat" ) );
	
				try
				{
					listener->lookupTransform("/cam_plane_flat", "/ovh_height", ros::Time(0), floor_to_cam);
				}
				catch( tf::TransformException &ex )
				{
					ROS_WARN( "unable to do transformation: [%s]", ex.what());
					continue;
				}

				// get position of red dot 
				cv::Point2d uv;
	      cv::Point2d uvrect;
    	  cv::Point3d red3d, yellow3d;

				uv.x = rb.x + rb.width/2.0; uv.y = rb.y + rb.height/2.0;
      	pcam.rectifyPoint( uv, uvrect ); 
  	    pcam.projectPixelTo3dRay( uvrect, red3d );
      
	      tf::Vector3 red3d_rel_cam (red3d.x, red3d.y, red3d.z);
      	tf::Vector3 red3d_uv ( floor_to_cam * red3d_rel_cam );

    	  // project to correct distance from camera
  	    cv::Point3d red_pt (ir_plane * red3d_uv.x() / red3d_uv.z(), ir_plane * red3d_uv.y() / red3d_uv.z(), ir_plane);
				// x,y is proj_pt.x, proj_pt.y

				uv.x = yb.x + yb.width/2.0; uv.y = yb.y + yb.height/2.0;
      	pcam.rectifyPoint( uv, uvrect ); 
  	    pcam.projectPixelTo3dRay( uvrect, yellow3d );

	      tf::Vector3 yellow3d_rel_cam (yellow3d.x, yellow3d.y, yellow3d.z);
      	tf::Vector3 yellow3d_uv ( floor_to_cam * yellow3d_rel_cam );

    	  // project to correct distance from camera
  	    cv::Point3d yellow_pt (ir_plane * yellow3d_uv.x() / yellow3d_uv.z(), ir_plane * yellow3d_uv.y() / yellow3d_uv.z(), ir_plane);


				// get orientation of red dot

				double yaw = atan2( yellow_pt.y-red_pt.y, yellow_pt.x-red_pt.x );

				tf::Transform robot_pub;
				ROS_DEBUG( "height: %0.2f floor_to_cam: %0.2f ir_plane: %0.2f", height, floor_to_cam.getOrigin().z(), ir_plane );
				robot_pub.setOrigin( tf::Vector3(red_pt.x, red_pt.y, ir_plane ));//floor_to_cam.getOrigin().z()) );
				robot_pub.setRotation( tf::Quaternion(yaw,0,0));

				odom.header.stamp = yellows_->header.stamp;

				odom.pose.pose.position.x = red_pt.x;
				odom.pose.pose.position.y = red_pt.y;

				odom.pose.pose.orientation.x = robot_pub.getRotation().getX();
				odom.pose.pose.orientation.y = robot_pub.getRotation().getY();
				odom.pose.pose.orientation.z = robot_pub.getRotation().getZ();
				odom.pose.pose.orientation.w = robot_pub.getRotation().getW();

				odom_pub.publish( odom );

				tb->sendTransform(tf::StampedTransform(robot_pub, yellows_->header.stamp, "cam_plane_flat", "pr2_ovh_target" ));

			}
		}
		else if( last_time > yellows_->header.stamp )
		{
			last_time = yellows_->header.stamp;
		}
	
	}
  return 0;
}

