/*
 * pr2_odom_avg
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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
int good_values_ = 0;

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "pr2_odom" );
	ros::NodeHandle nh;
	tf::TransformListener tl;
	tf::TransformBroadcaster tb;
	ros::Rate loop_rate( 10 );

	tf::Transform avg_tf;

	double sx = 0,sy = 0,sz = 0, sqx = 0, sqy = 0, sqz = 0, sqw = 0;

	while( ros::ok() )
	{
		if( good_values_ < 50 )
		{
			try {
				tf::StampedTransform new_tf;
				tl.lookupTransform( "ovh", "map", ros::Time(0), new_tf );
				sx += new_tf.getOrigin().x();
				sy += new_tf.getOrigin().y();
				sz += new_tf.getOrigin().z();
				sqx += new_tf.getRotation().getX();
				sqy += new_tf.getRotation().getY();
				sqz += new_tf.getRotation().getZ();
				sqw += new_tf.getRotation().getW();
				good_values_++;

				if( good_values_ == 50 )
				{
					avg_tf.setOrigin( tf::Vector3(sx / 50., sy / 50., sz / 50.) );
					avg_tf.setRotation( tf::Quaternion( sqx / 50., sqy / 50., sqz / 50., sqw / 50. ) );
				}
			}
			catch( tf::TransformException &ex )
			{
				ROS_WARN( "could not do transform: [%s]", ex.what() );
			}
		}
		else
		{
			// publish transform
			tb.sendTransform(tf::StampedTransform(avg_tf,ros::Time::now(), "ovh", "map" ));
		}

		loop_rate.sleep();
		ros::spinOnce();
	}


	return 0;
}
