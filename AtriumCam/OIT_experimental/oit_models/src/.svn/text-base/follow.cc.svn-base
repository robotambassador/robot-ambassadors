#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <p2os_driver/MotorState.h>

bool running = true;

#define MAX_SPD 0.1

void cmd_cb( const p2os_driver::MotorStateConstPtr &cmd )
{
	if( cmd->state == 0 ) running = false;
	else running = true;
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "follower" );
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::Rate loop_rate(10);

	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = 0;

	std::string child_frame;
	std::string robot_frame;

	nh_priv.param( "child_frame", child_frame, std::string("/child/base_link"));
	nh_priv.param( "robot_frame", robot_frame, std::string("/robot/base_link"));

	tf::TransformListener tl;

	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

	ros::Subscriber cmd_sub = nh.subscribe("cmd_motor_state", 1, cmd_cb );

	while( ros::ok() )
	{

		tf::StampedTransform cr;
		try {
			tl.lookupTransform( child_frame, robot_frame, ros::Time(0), cr );
		}
		catch( tf::TransformException &ex )
		{
			ROS_INFO( "could not do transformation: [%s]", ex.what() );
			continue;
		}

		double xdir = cr.getOrigin().x() > 0 ? 1 : -1;
		double ydir = cr.getOrigin().y() > 0 ? 1 : -1;

		double dx = fabs(cr.getOrigin().x())-(sqrt(2.0)/2.0);
		double dy = fabs(cr.getOrigin().y())-(sqrt(2.0)/2.0);

		if( dx < 0 ) dx = 0;
		if( dy < 0 ) dy = 0;

		dx *= xdir;
		dy *= ydir;

		if( dx >  MAX_SPD ) dx = MAX_SPD;
		if( dx < -MAX_SPD ) dx = -MAX_SPD;
		if( dy >  MAX_SPD ) dy = MAX_SPD;
		if( dy < -MAX_SPD ) dy = -MAX_SPD;
	
		if( running )
		{
			cmd_vel.linear.x = dx;
			cmd_vel.linear.y = dy;
		}
		else
		{
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
		}

		cmd_vel_pub.publish( cmd_vel );

		loop_rate.sleep();
		ros::spinOnce();
	}

	ros::spinOnce();
	ros::spinOnce();

	ros::Duration(1.0).sleep();

	ros::spinOnce();
	ros::spinOnce();

	return 0;
}
