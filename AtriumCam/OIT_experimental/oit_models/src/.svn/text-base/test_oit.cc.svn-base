#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <oit_models/LookupProb.h>

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "test_oit" );
	ros::NodeHandle nh;
	tf::TransformListener tl;

	std::string global_frame = "/ovh";

	ros::Rate loop_rate(10);
	while( ros::ok() )
	{
		loop_rate.sleep();
		ros::spinOnce();

		// for each tick get the rx,ry pose of the robot
		tf::StampedTransform robot_tf;
		try 
		{
			tl.waitForTransform( "/robot/base_link", global_frame, ros::Time(0), ros::Duration(5.0));
			tl.lookupTransform( global_frame, "/robot/base_link", ros::Time(0), robot_tf );
		}
		catch( tf::TransformException &ex )
		{
			ROS_WARN( "could not do transformation: [%s]", ex.what() );
			continue;
		}

		oit_models::LookupProb::Request  req;
		oit_models::LookupProb::Response res;

		req.robot_pos.x = robot_tf.getOrigin().x();
		req.robot_pos.y = robot_tf.getOrigin().y();
		req.robot_pos.z = 0;

		if( !ros::service::call("/lookup_prob", req, res ) )
		{
			ROS_WARN( "lookup_prob service call failed" );
		}

		ROS_INFO( "x_i: %4.6f y_i: %4.6f (%0.2f)", robot_tf.getOrigin().x(), robot_tf.getOrigin().y(), res.prob );
	}

	return 0;
}
