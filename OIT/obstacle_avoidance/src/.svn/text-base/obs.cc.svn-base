#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "point_map/PointMap.h"
#include "oit_msgs/ObstacleArray.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <vector>
#include "line.hpp"
#include "raytrace.hpp"

#define DTOR( a ) a * M_PI / 180.0 
#define RTOD( a ) a * 180.0 / M_PI
#define FSPAN 0.2

geometry_msgs::Twist     cmd_vel_;
geometry_msgs::Twist     obs_vel_;
oit_msgs::ObstacleArray obs_;
std::vector<ObjectPtr>  scene_;  

double rx, ry, rt;
double _lvelspeed, _rvelspeed, _min_speed;

bool notified_tf = false;

bool got_map;
point_map::PointMap::Response map;
sensor_msgs::LaserScan laserscan;
sensor_msgs::PointCloud cloud;
tf::TransformListener* tl;
tf::TransformBroadcaster *tb;

ros::Publisher poseDot_pub;
ros::Publisher laserScan_pub;
ros::Publisher sonarHit_pub;
ros::Publisher cloud_pub;
laser_geometry::LaserProjection projector;

std::vector<std::string> frame_strings;

void
get_map()
{
  point_map::PointMap::Request  req;
  if( ros::service::call("/map_server", req,map) )
  {
    got_map = true;
  }
} 

void
cmdvel_cb( const geometry_msgs::TwistConstPtr& msg)
{
  cmd_vel_ = *msg;
}

void
obs_cb( const oit_msgs::ObstacleArrayConstPtr& msg)
{
  obs_ = *msg;
}

void
runloop()
{
  double odist = 0.15;

  /* robot pose relative to ovh frame */
	std::vector<tf::StampedTransform> transforms;
  tf::StampedTransform rtrans;

  try {
    tl->lookupTransform("/map","/robot/base_link", ros::Time(0), rtrans );
		
		for( unsigned int i = 0; i < frame_strings.size(); i++ )
			if( tl->canTransform( "/map",frame_strings[i], ros::Time(0) ) )
			{
				tf::StampedTransform tr;
		    tl->lookupTransform("/map",frame_strings[i], ros::Time(0), tr );
				transforms.push_back( tr );
			}
  }
  catch (tf::TransformException  &ex ) {
    if( !notified_tf )
    {
      ROS_WARN( "unable to do transformation: [%s]\n", ex.what());
      notified_tf = true;
    }
  }

  rx = rtrans.getOrigin().x();
  ry = rtrans.getOrigin().y();
  double pitch,roll;
  rtrans.getBasis().getEulerYPR(rt,pitch,roll);

	//ROS_INFO( "@" );

  if( !got_map ) get_map();

	//ROS_INFO( "%%" );

  // add map walls
  if( got_map )
  {
    int num_points = map.num_points;
    for( int i = 0; i < num_points; i++ )
    {
      scene_.push_back(LinePtr(
                        new Line(
                          map.x[i],map.x[(i+1)%num_points],
                          map.y[i],map.y[(i+1)%num_points])));
    }
  }

	//ROS_INFO( "$" );

  // obstacles
  for( int i = 0; i < obs_.num_obstacles; i++ )
  {
    double p1x = obs_.obstacles[i].x-obs_.obstacles[i].width/2.0;
    double p2x = obs_.obstacles[i].x+obs_.obstacles[i].width/2.0;
    double p1y = obs_.obstacles[i].y-obs_.obstacles[i].height/2.0;
    double p2y = obs_.obstacles[i].y+obs_.obstacles[i].height/2.0;
    scene_.push_back( LinePtr( new Line( p1x, p1x, p1y, p2y )));
    scene_.push_back( LinePtr( new Line( p1x, p2x, p1y, p1y )));
    scene_.push_back( LinePtr( new Line( p1x, p2x, p2y, p2y )));
    scene_.push_back( LinePtr( new Line( p2x, p2x, p1y, p2y )));
  }

  // add fake sonar hits
	for( unsigned int i = 0; i < transforms.size(); i++ )
	{
    double cx = transforms[i].getOrigin().x();
    double cy = transforms[i].getOrigin().y();

    double range = hypot( cy-ry, cx-rx );
    double bearing = RTOD( atan2( cy-ry, cx-rx ) );

    if( !(range < odist && bearing > -45 && bearing < 45 ) )
      scene_.push_back(LinePtr(new Line(cx-odist,cx-odist,cy-odist,cy+odist)));
    if( !(range < odist && (bearing > 135 || bearing < -135) ) )
      scene_.push_back(LinePtr(new Line(cx+odist,cx+odist,cy-odist,cy+odist)));
    if( !(range < odist && bearing < 135 && bearing > 45 ) )
      scene_.push_back(LinePtr(new Line(cx-odist,cx+odist,cy-odist,cy-odist)));
    if( !(range < odist && bearing > -135 && bearing < -45 ) )
      scene_.push_back(LinePtr(new Line(cx-odist,cx+odist,cy+odist,cy+odist)));
  }

  // get flbr values
  float fdist = 9999, ldist = 9999, rdist = -9999, bdist = -9999;

  float RANGE = 100000;

  std::vector<float> depths;
  float step = 3;

  //ROS_INFO( "^" );

  for( float i = -180; i < 180; i += step )
  {
    float ri = DTOR( i );
    float rx2 = rx + RANGE * cos( rt + ri);
    float ry2 = ry + RANGE * sin( rt + ri);
    
    RayPtr curRay = RayPtr( new Ray( rx, ry, 0, rx2, ry2, 0));
    float closest = FLT_MAX;
    ObjectPtr closestObj;
    float t = FLT_MAX;
    for( std::vector<ObjectPtr>::const_iterator ii = scene_.begin(); ii!= scene_.end(); ++ii )
    {
      t = (*ii)->getIntersection(curRay);
      if( t > 0 && t < closest )
      {
        closest = t;
        closestObj = (*ii);
      }
    }

		if( closest > 100 ) closest = 100;

    depths.push_back(closest);
  }
  float ii = -180;

  // build laser_msg
  laserscan.angle_min = DTOR(-180);
  laserscan.angle_max = DTOR(179);
  laserscan.angle_increment = DTOR(step);
  laserscan.range_min = 0.0;
  laserscan.range_max = 8.192;
  laserscan.ranges.resize((int)(360/step));
  laserscan.intensities.resize((int)(360/step));
  laserscan.header.frame_id = "/robot/base_laser";
  laserscan.header.stamp = rtrans.stamp_;

	//ROS_INFO( "+" );

  for( std::vector<float>::const_iterator i = depths.begin(); i != depths.end(); ++i, ii+=step )
  {
    float ri = DTOR( ii );
    float ox = rx+*i*cos(rt+ri);
    float oy = ry+*i*sin(rt+ri);

    int iii = (int) ((ii+180)/step);

		geometry_msgs::Point point;
		point.x = (double) ox;
		point.y = (double) oy;
		point.z = 0;
		//ROS_INFO( "x: %0.2f y: %0.2f\n", ox, oy );
		
    double r = *i;
    //printf( "iii: %d\n", iii );
    laserscan.ranges[iii] = r;
    laserscan.intensities[iii] = 0.0;
    //printf( "iii: %d\n", iii );
    if( *i > 300 ) continue;
    //double b = ri;

/************************** flbr ****************/
/*
    double sx = r * cos(ri);
    double sy = r * sin(ri);

    //if( fabs( RTOD( ri )) < 20.0 )
    //printf( "point (%f,%f:%f): (%f,%f)\n", r, RTOD(b), RTOD(ri), sx, sy );
    if( sx > 0.0 )
    {
      if( sy <= FSPAN && sy >= -FSPAN )
        if( sx <= fdist )
        {
          fdist = sx;
        }
      if( sx < 0.75 && sx > 0.2 )
      {
        if( sy >= 0.25 && sy < ldist ) ldist = sy; //else ldist = 9999;
        if( sy < -0.15 && sy > rdist ) rdist = sy; //else rdist = -9999;
      }
    }
    if( sx < 0.0 )
      if( sy <= FSPAN && sy >= -FSPAN )
        if( sx >= bdist )
          bdist = sx;

*/
/************************** flbr ****************/
  }
 
  scene_.clear();

/************************** flbr ****************/
/*

  // set velocity based on flbr values
  obs_vel_.linear.x = cmd_vel_.linear.x;
  obs_vel_.angular.z = cmd_vel_.angular.z;
	//ROS_INFO( "-" );

  if( cmd_vel_.linear.x > 0 )
  {
    if( fdist < 1.5 ) //&& _state != MOVE_STATE_APPROACH && _state != MOVE_STATE_ROCK && _state != MOVE_STATE_WIGGLE  )
    {
      //if( ldist +rdist >  0.0 ) obs_vel_.angular.z = (1.5-fdist) / 4.0 * _rvelspeed;
      //if( ldist +rdist <= 0.0 ) obs_vel_.angular.z = -(1.5-fdist) / 4.0 *_rvelspeed;
    }
    if( fdist < 1.0 ) obs_vel_.linear.x = cmd_vel_.linear.x * (fdist / 1.0);
    if( obs_vel_.linear.x < _min_speed ) 
    {
      //printf( "MIN_SPEED: %f\n", obs_vel_.linear.x );
      obs_vel_.linear.x = _min_speed;
    }
  }
  if( obs_vel_.linear.x < 0 )
  {
    if( bdist > -0.5 ) obs_vel_.linear.x = 0.0;
    if( obs_vel_.linear.x > -_min_speed )
    {
      //printf( "MIN_SPEED: %f\n", obs_vel_.linear.x );
      obs_vel_.linear.x = -_min_speed;
    }
  }

  if( fdist < 0.2 ) {
    if( bdist < -0.5 ) obs_vel_.linear.x = -_min_speed;
    // DFS: rvel fix 2/4 undone on 2/5
      obs_vel_.angular.z = 0;
  }

  ROS_DEBUG( "(%0.2f|%0.2f|%0.2f|%0.2f) (%0.2f,%0.2f) ==> (%0.2f,%0.2f)", fdist, ldist, rdist, bdist, cmd_vel_.linear.x, cmd_vel_.angular.z, obs_vel_.linear.x, obs_vel_.angular.z );

  poseDot_pub.publish(obs_vel_);
*/
/************************** flbr ****************/


  laserScan_pub.publish(laserscan);
  tf::Transform t(tf::Quaternion(0,0,0,1),tf::Point(0,0,0));

	// TODO: make more general (specify as param strings 
  tf::StampedTransform ti( t, laserscan.header.stamp, "/robot/base_link", "/robot/base_laser" );
  tb->sendTransform(ti);
}


int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "obs" );
  ros::NodeHandle n;
	ros::NodeHandle nh_priv("~");
  ros::Rate loop_rate(20);

  laserScan_pub = n.advertise<sensor_msgs::LaserScan>("/robot/laser", 1000);
  ros::Subscriber obs_sub = n.subscribe("/obstacles",1,obs_cb);

/**************************** flbr *********************/
/*
  poseDot_pub = n.advertise<geometry_msgs::Twist>("/robot/des_vel",1000); 
  ros::Subscriber cmd_sub = n.subscribe("des_vel",1,cmdvel_cb);
  
  n.param( "lvel", _lvelspeed, 0.25 );
  n.param( "rvel", _rvelspeed, 0.75 );
  n.param( "min_speed", _min_speed, 0.15 );
*/
/**************************** flbr *********************/

	// parse frame string as list of frames delimited by commas

	std::string frame_string;
	nh_priv.param( "frames", frame_string, std::string("/child/base_link,/parent/base_link") );
	frame_strings.empty();
  std::string::size_type i = 0;
  std::string::size_type j = frame_string.find(',');
	
  if( j == std::string::npos )
  {
    std::string ss = frame_string.substr(i,frame_string.length());
    frame_strings.push_back(ss);
  }

  while( j != std::string::npos )
  {
    std::string s = frame_string.substr(i,j-i);
    frame_strings.push_back(s);

    i = ++j;
    j = frame_string.find(',', j);
    if( j == std::string::npos )
    {
      std::string ss = frame_string.substr(i,frame_string.length());
      frame_strings.push_back(ss);
    }
  }


  got_map = false;

  obs_.num_obstacles = 0;
  tl = new tf::TransformListener(n);
  tb = new tf::TransformBroadcaster();

	ros::Duration(1.0).sleep();

  while( ros::ok() )
  {
		ros::spinOnce();

		// TODO: change to a system where laserscan is updated only when needed
		// (1): when obstacle message received
		// (2): when new tf message received

    runloop();
    loop_rate.sleep();
  }

  return 0;
}
