/*
 * floor_align
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>


tf::TransformListener * tl;
tf::TransformBroadcaster * tb;
tf::StampedTransform * raw_frame;
tf::Transform * filtered_frame;
tf::Transform * floor_norm_to_cam;
tf::Transform * result; //cam to floor norm

ros::Rate * loop_rate;

int numSamples;

bool getSample()
{
	static ros::Time lastTime = ros::Time(0);
	try
	{
		tl->waitForTransform("/ovh_height", "/floor", lastTime, ros::Duration(5.0));
		tl->lookupTransform("/ovh_height", "/floor", lastTime, (*raw_frame));
		
		//make sure we set a delay so we don't just grab the same frame n times
		lastTime = raw_frame->stamp_;
		lastTime += ros::Duration(0.1);
		
		fprintf(stdout, "x %f y %f z %f\n", raw_frame->getOrigin().x(), raw_frame->getOrigin().y(), raw_frame->getOrigin().z());
		
		//using filtered_frame as the sum of all the frames
		filtered_frame->setOrigin(filtered_frame->getOrigin() + raw_frame->getOrigin());
		filtered_frame->setRotation(filtered_frame->getRotation() + raw_frame->getRotation());
		
		//fprintf(stdout, "x %f y %f z %f\n", filtered_frame->getOrigin().x(), filtered_frame->getOrigin().y(), filtered_frame->getOrigin().z());
		return true;
	}   
	catch( tf::TransformException &ex )
	{
		ROS_WARN( "unable to do transformation: [%s]", ex.what());
	}
	return false;
}

void publishFilteredTransform()
{
	tb->sendTransform(tf::StampedTransform((*result), ros::Time::now(), "/ovh_height", "/ovh" ));
}

//wait for the specified frame and store it in raw_frame
bool lookupTransform(std::string frame1, std::string frame2)
{
	fprintf( stdout, "looking up transform from %s to %s...\n", frame1.c_str(), frame2.c_str() );
	try
	{
		tl->waitForTransform(frame1.c_str(), frame2.c_str(), ros::Time(0), ros::Duration(5.0));
		tl->lookupTransform(frame1.c_str(), frame2.c_str(), ros::Time(0), (*raw_frame));
		
		btScalar oriy, orip, orir;
		raw_frame->getBasis().getEulerYPR(oriy, orip, orir);
		
		fprintf( stdout, "found: pos <x %f y %f z %f> | ori <y %f p %f r %f>\n", raw_frame->getOrigin().x(), raw_frame->getOrigin().y(), raw_frame->getOrigin().z(), oriy, orip, orir );
		return true;
	}   
	catch( tf::TransformException &ex )
	{
		ROS_WARN( "unable to do transformation: [%s]", ex.what());
		ROS_WARN( "frame from %s to %s is not being published properly. exiting...", frame1.c_str(), frame2.c_str() );
	}
	return false;
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "floor_align" );
	ros::NodeHandle node;
	node.param("num_samples", numSamples, 5);
	
	tl = new tf::TransformListener(node);
	tb = new tf::TransformBroadcaster();
	
	//grabbing these
	raw_frame = new tf::StampedTransform;
	
	//publishing these
	floor_norm_to_cam = new tf::Transform;
	
	//calculating these (temp use)
	filtered_frame = new tf::Transform;
	filtered_frame->setOrigin( tf::Vector3( 0, 0, 0 ) );
	
	result = new tf::Transform;
	
	loop_rate = new ros::Rate(100);
	
	for(int i = 0; i < numSamples; i ++)
	{
		if(!getSample())
    	{
			ROS_WARN( "frame from /floor to /ovh_height is not being published properly. exiting..." );
			return 1;
    	}
		ros::spinOnce();
	}
	
	//vector to use when averaging the origins for the sampled frames
	tf::Vector3 scaledVec ( (float)numSamples, (float)numSamples, (float)numSamples );
	
	//average the origins of the sampled frames
	filtered_frame->setOrigin(filtered_frame->getOrigin() / scaledVec);
	
	//average the rotations of the sampled frames
	filtered_frame->setRotation(filtered_frame->getRotation() / (float) numSamples);
	
	//center of the checkerboard
	tf::Vector3 * theOrigin = & ( filtered_frame->getOrigin() );
	
	fprintf(stdout, "origin: x %f y %f z %f\n", theOrigin->x(), theOrigin->y(), theOrigin->z());
	
	tf::Vector3 floorNormalVec (0, 0, 1);
	
	if( !lookupTransform("/floor", "/ovh_height") )
		return 1;
		
	tf::Vector3 floorToCam = raw_frame->getOrigin();
		
	floor_norm_to_cam->setOrigin( tf::Vector3( raw_frame->getOrigin().x(), raw_frame->getOrigin().y(), 0 ) );
	tf::Quaternion theAngle; theAngle.setEuler( 0.0f, 0.0f, 0.0f ); //only did this to kill compiler warnings
	floor_norm_to_cam->setRotation( theAngle );
	
	tb->sendTransform(tf::StampedTransform((*floor_norm_to_cam), ros::Time::now(), "/floor", "/ovh_tmp" ));
	
	float theta = floorToCam.angle( floorNormalVec );
	
	float height = filtered_frame->getOrigin().length() * cos( theta );
	
	fprintf( stdout, "theta: %f\n", theta );
	fprintf( stdout, "height: %f\n", height );
	
	if( !lookupTransform("/ovh_height", "/ovh_tmp") )
		return 1;
	
	*result = *raw_frame;
	
	*theOrigin = raw_frame->getOrigin();
	btScalar oriy, orip, orir;
	raw_frame->getBasis().getEulerYPR( oriy, orip, orir );
	
	ROS_INFO("<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"floor_align_pub\" args=\"%f %f %f %f %f %f /ovh_height /ovh 100\" />\n", theOrigin->x(), theOrigin->y(), theOrigin->z(), oriy, orip, orir );
	
	while( node.ok() )
	{
		publishFilteredTransform();
		
		ros::spinOnce();
		loop_rate->sleep();
	}
	
	ros::spin();
	
	return 0;
}
