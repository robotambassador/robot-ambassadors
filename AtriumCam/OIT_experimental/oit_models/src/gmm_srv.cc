#include "gmm.h"
#include <ros/ros.h>
#include "oit_models/GMMClassify.h"
#include "oit_models/LookupProb.h"
#include <tf/transform_listener.h>

GMM g;

tf::TransformListener *tf_;
std::string child_frame_name;
double init_robot_dist;
double init_child_dist;
geometry_msgs::PoseStamped curr_goal, updated_goal;
std::string robot_frame, global_frame, child_frame;
tf::StampedTransform child_tf;


void goal_cb( const geometry_msgs::PoseStampedConstPtr &goal )
{
  ROS_INFO( "goal sensed" );

  curr_goal.header = goal->header;
  curr_goal.pose = goal->pose;

  // get initial distance of robot to goal...
  try {
    //tf_->waitForTransform( global_frame, "/odom_combined", curr_goal.header.stamp, ros::Duration(1.0) );
    geometry_msgs::PoseStamped local_pose, child_goal;
    tf_->transformPose( global_frame, ros::Time(0), curr_goal, global_frame, local_pose );
    tf::StampedTransform rf, cf;
    tf_->lookupTransform( global_frame, robot_frame, ros::Time(0), rf );
    tf_->lookupTransform( global_frame, child_frame, ros::Time(0), cf );
    curr_goal.pose = local_pose.pose;
    curr_goal.header = local_pose.header;
    init_robot_dist = hypot( local_pose.pose.position.y-rf.getOrigin().y(), local_pose.pose.position.x-rf.getOrigin().x() );
		ROS_DEBUG( "(%0.2f,%0.2f) (%0.2f,%0.2f)", local_pose.pose.position.x,local_pose.pose.position.y,rf.getOrigin().x(),rf.getOrigin().y() );
    init_child_dist = hypot( local_pose.pose.position.y-cf.getOrigin().y(), local_pose.pose.position.x-cf.getOrigin().x() );
  }
  catch (tf::TransformException &ex )
  {
    ROS_WARN( "could not do transformation: [%s]\n", ex.what() );
  }
}

bool classify(oit_models::GMMClassify::Request  &req,
				 			oit_models::GMMClassify::Response &res )
{
	// infer time from robot distance
  double p1 = -1.0676;
  double p2 =  1.4255;
  double p3 = -1.2337;
  double p4 =  1.0497;

  //double t = -0.86466*req.data[1] + 1.0661;
  double x = req.data[1];
  double t = p1 * x*x*x + p2 * x*x + p3 * x + p4;

	if( t > 1. ) t = 1.;

	std::vector<double> data;
	data.push_back(t);
	for( int i = 1; i < req.data.size(); i++ )
	{
		data.push_back(req.data[i]);
	}

	//ROS_INFO( "<<< %0.2f %0.2f %0.2f %0.2f >>>", data[0], data[1], data[2], data[3] );

	double mahal = g.prob( &data );
	res.res = mahal;

	
	// convert mahal to prob
	double prob = 0.0;
	x = 0.0;

	if( fabs (mahal)  < 1e-6 ) 
		x = 0;
	else
	{
		double y =0.5 * fabs(mahal);
		if( y > (6*0.5) ) {
			x = 1.0;
		}
		else if( y < 1.0 )
		{
			double w = y*y;
			x = ((((((((0.000124818987 * w
                - 0.001075204047) * w + 0.005198775019) * w
                - 0.019198292004) * w + 0.059054035642) * w
                - 0.151968751364) * w + 0.319152932694) * w
                - 0.531923007300) * w + 0.797884560593) * y * 2.0;
		}
		else
		{
			y = -2.0;
      x = (((((((((((((-0.000045255659 * y
                      + 0.000152529290) * y - 0.000019538132) * y
                      - 0.000676904986) * y + 0.001390604284) * y
                      - 0.000794620820) * y - 0.002034254874) * y
                      + 0.006549791214) * y - 0.010557625006) * y
                      + 0.011630447319) * y - 0.009279453341) * y
                      + 0.005353579108) * y - 0.002141268741) * y
                      + 0.000535310849) * y + 0.999936657524;

		}
	}

	//rob = ((1.0-x));	
	prob = x;
	//if( prob > 1 ) prob = 0;
	res.res = mahal;
	return true;
}


bool lookup_prob(oit_models::LookupProb::Request  &req,
                 oit_models::LookupProb::Response &res )
{
  double rx, ry, cx, cy, gx, gy;
  rx = req.robot_pos.x;
  ry = req.robot_pos.y;

  cx = child_tf.getOrigin().x();
  cy = child_tf.getOrigin().y();

  gx = updated_goal.pose.position.x;
  gy = updated_goal.pose.position.y;

  double robot_dist, child_dist, robot_child_dist;
  robot_dist = hypot( rx-gx, ry-gy );
  child_dist = hypot( cx-gx, cy-gy );
  robot_child_dist = hypot( cx-rx, cy-ry );

	ROS_DEBUG( "(%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f) [%0.2f %0.2f]", rx, ry, cx, cy, gx, gy, init_robot_dist, init_child_dist );

  // scale all values to initial values
  robot_dist /= init_robot_dist;
  child_dist /= init_child_dist;

  oit_models::GMMClassify::Request gmm_req;
  gmm_req.data.push_back(0.0);
  gmm_req.data.push_back(robot_dist);
  gmm_req.data.push_back(child_dist);
  gmm_req.data.push_back(robot_child_dist);

	oit_models::GMMClassify::Response gmm_res;

	classify( gmm_req, gmm_res );
 
  res.prob = gmm_res.res;
  return true;
}

int main( int argc, char* argv[] )
{

	ros::init( argc, argv, "gmm_srv" );
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::AsyncSpinner as(50);

  tf_ = new tf::TransformListener(ros::Duration(100.0));
	tf_->setExtrapolationLimit( ros::Duration(1.0));

	ros::ServiceServer service = nh.advertiseService( "gmm_classifier", classify );
  ros::ServiceServer other_service = nh.advertiseService("lookup_prob", lookup_prob );
  nh_priv.param( "child_frame", child_frame, std::string("/child/base_link"));
  //nh_priv.param( "robot_frame", robot_frame, std::string("/robot/base_link"));
  //nh_priv.param( "global_frame", global_frame, std::string("/ovh"));
  nh_priv.param( "robot_frame", robot_frame, std::string("/base_link"));
  nh_priv.param( "global_frame", global_frame, std::string("base_footprint"));

  ros::Subscriber goal_sub = nh.subscribe( "goal", 1, goal_cb );

	std::string filename;
	std::string prefix;
	nh_priv.param( "filename", filename, std::string("moves.files") );
	nh_priv.param( "prefix", prefix, std::string(""));
	g.load_data_from_file( filename.c_str(), prefix.c_str() );

	std::vector<int> used_dims;
	used_dims.push_back(0);
	used_dims.push_back(9);
	used_dims.push_back(10);
	used_dims.push_back(11);

	g.train_model( 3, &used_dims );

	ros::Rate loop_rate( 100 );

	as.start();

	while( ros::ok() )
	{
		char c = 0;
		c = cv::waitKey(10);
	  // lookup transform info
  	try {
			tf_->waitForTransform( child_frame, global_frame, ros::Time(0), ros::Duration(5.0) );
    	tf_->lookupTransform( child_frame, global_frame, ros::Time(0), child_tf );
      tf_->transformPose( global_frame, ros::Time(0), curr_goal, global_frame, updated_goal );
	  }
	  catch (tf::TransformException &ex )
  	{
  	  ROS_WARN( "could not do transformation: [%s]", ex.what() );
	  }

		loop_rate.sleep();
	}
	return 0;
}
