#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>

#include <geometry_msgs/PoseStamped.h>

navfn::NavfnROS navfn_plan;
ros::Subscriber goal_sub;

tf::TransformListener* tfl;

void goal_cb( const geometry_msgs::PoseStampedConstPtr &msg )
{
	geometry_msgs::PoseStamped tmsg;

	tfl->transformPose( "/map", *msg, tmsg );
	geometry_msgs::PoseStamped start = tmsg, tstart;
	start.header.frame_id = "/robot/odom";
	start.pose.position.x = 0;
	start.pose.position.y = 0;
	start.pose.position.z = 0;
	tfl->transformPose( "/map", start, tstart );

	ROS_INFO( "plan(%s): (%f,%f) ==> (%f,%f)", tmsg.header.frame_id.c_str(), tstart.pose.position.x,tstart.pose.position.y,tmsg.pose.position.x,tmsg.pose.position.y );

	std::vector< geometry_msgs::PoseStamped > path;

	bool planned = navfn_plan.makePlan( tstart, tmsg, path );
	if( !planned )
	{
		ROS_WARN( "plan did not succeed" );
	}
/*
	for( unsigned int i = 0; i < path.size(); i++ )
	{
		ROS_INFO( "path(%d): (%f,%f)", i, path[i].pose.position.x, path[i].pose.position.y );
	}
*/
	navfn_plan.publishPlan( path, 0.0, .8, 0.0, 0.2 );
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "overhead_planner" );
	ros::NodeHandle nh("~");	

	tfl = new tf::TransformListener(ros::Duration(10));
	costmap_2d::Costmap2DROS costmap("ovh_costmap", *tfl );
	navfn_plan.initialize("ovh_navfn_planner", &costmap );

	goal_sub = nh.subscribe("/goal", 1, goal_cb );

	ros::spin();
	return 0;
}
