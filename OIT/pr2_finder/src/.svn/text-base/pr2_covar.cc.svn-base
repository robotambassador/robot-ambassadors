#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>

bool g_stopped = false;

#define MX .000001

double stddev( std::vector<double> vec )
{
	double sum = 0;
	for( unsigned int i= 0; i < vec.size(); i++ )
	{ 
		sum += vec[i]*vec[i];
	}
	return sqrt(sum);
}

void cmd_vel_sub( const geometry_msgs::TwistConstPtr &cmdvel )
{
	bool stopped = true;

	if( fabs(cmdvel->linear.x) > MX || fabs(cmdvel->linear.y) > MX || fabs(cmdvel->linear.z) > MX ) stopped = false;
	if( fabs(cmdvel->angular.x) > MX || fabs(cmdvel->angular.y) > MX || fabs(cmdvel->angular.z) > MX ) stopped = false;

	g_stopped = stopped;
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "pr2_covar" );
	ros::NodeHandle nh;
	tf::TransformListener tl;
	ros::Subscriber cmd_sub = nh.subscribe("/base_controller/command", 1, cmd_vel_sub);

	ros::Rate loop_rate(15);

	bool was_stopped = false;

	std::vector<double> x, y, theta;
	std::vector<double> ax, ay, atheta;
	std::vector<double> normx, normy, normtheta;

	while( ros::ok() )
	{
		// if stopped
		if( g_stopped )
		{
			tf::StampedTransform rtrans;
			if( !was_stopped ) 
			{
				ROS_INFO( "stopped" );
				x.clear(); y.clear(); theta.clear();
			}
			// lookup current transform
			try {
				tl.lookupTransform( "/ovh", "/pr2_ovh_target", ros::Time(0), rtrans );
				x.push_back( rtrans.getOrigin().x() );
				y.push_back( rtrans.getOrigin().y() );
				double roll, pitch, yaw;
				rtrans.getBasis().getEulerYPR( yaw, pitch, roll );
				theta.push_back(yaw);
			}
			catch( tf::TransformException &ex )
			{
				ROS_WARN( "could not do transform: [%s]", ex.what() );
			}
		}
		// else
		else
		{
			if( was_stopped ) 
			{
				ROS_INFO( "moving" );
				// get differences from mean for cached values

				double meanx = 0, meany = 0, meant = 0;
				for( unsigned int i = 0; i < x.size(); i++ )
				{
					meanx += x[i];
					meany += y[i];
					meant += theta[i];
				}

				meanx /= x.size();
				meany /= x.size();
				meant /= x.size();

				for( unsigned int i = 0; i < x.size(); i++ )
				{
					ax.push_back( x[i] / meanx );
					ay.push_back( y[i] / meany );
					atheta.push_back( theta[i] / meant );
				}
			}
		}
		// print out variances
		double varx, vary, vart;
		varx = stddev( ax );
		vary = stddev( ay );
		vart = stddev( atheta );

		ROS_INFO( "vx: %f vy: %f vt: %f", varx, vary, vart );

		was_stopped = g_stopped;
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
