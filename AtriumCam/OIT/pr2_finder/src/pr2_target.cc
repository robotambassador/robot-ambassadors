/*
 * pr2_target_tf_publisher
 * Copyright (c) 2011, David Feil-Seifer, University of Southern California
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
 *     * Neither the name of the University of Southern California nor the names of its
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "pr2_target_tf_publisher" );
	ros::NodeHandle nh;

	tf::TransformListener tl;
	tf::TransformBroadcaster tb;

	ros::Rate loop_rate( 20 );
	while( ros::ok() )
	{
		// publish frame for target

		tf::Transform target_tf;
		//target_tf.setOrigin( tf::Vector3( -.2032, 0.03095625, 0.0381 ) );
		target_tf.setOrigin( tf::Vector3( -.2032, 0.03095625, 0.0481 ) );
		target_tf.setRotation( tf::Quaternion( 0,0,0 ) );
		
		tb.sendTransform( tf::StampedTransform( target_tf, ros::Time::now(), "narrow_stereo_gazebo_r_stereo_camera_frame", "pr2_tmp_target" ) );

		tf::StampedTransform reverse_tf;

		// get the transform from the base to the top link of the pr2
		
		try
		{
			tl.lookupTransform( "pr2_tmp_target", "map", ros::Time(0), reverse_tf );
			tf::Transform reverse_target_tf;
			reverse_target_tf.setOrigin( reverse_tf.getOrigin() );
			reverse_target_tf.setRotation( reverse_tf.getRotation() );
			tb.sendTransform( tf::StampedTransform( reverse_target_tf, reverse_tf.stamp_, "pr2_ovh_target", "map" ));

		}
		catch (tf::TransformException &ex)
		{
			ROS_WARN("unable to do transformation: [%s]", ex.what());
		}
			
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

