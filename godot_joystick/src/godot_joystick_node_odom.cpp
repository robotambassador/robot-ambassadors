#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

static const double MAX_ANGULAR = 0.5; // rad/s
static const double MAX_LINEAR = 0.5; // m/s
static const double ANGULAR_THRESH = 5; 

class GodotJoystick
{
public:
  GodotJoystick();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  bool turnOdom(bool clockwise, double radians);
  
  ros::NodeHandle nh_;
  tf::TransformListener listener_;

  int l_linear_, l_angular_, r_linear_, r_angular_, l_trigger_, r_trigger_;

  int button_a_, button_b_, button_x_, button_y_, button_start_, button_back_, button_l_bumper_, button_r_bumper_;

  int d_axis_up_down_, d_axis_left_right_;

  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
	ros::Publisher brake_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber odom_sub_;
  
  
  double next_linear_;
  double next_angular_;
  int turn_angle_;
  int count_, loop_rate_, turn_time_;
  double angular_goal_;
  bool turning_;
};


GodotJoystick::GodotJoystick()
{
  nh_.param("turn_angle", turn_angle_, 30);

  nh_.param("scale_angular", a_scale_, -0.4);
  nh_.param("scale_linear", l_scale_, 0.3);

	//Joypad button mapping parameters
	//Defaults for Logitech F710 controller
  nh_.param("l_axis_linear", l_linear_, 1);
  nh_.param("l_axis_angular", l_angular_, 0);
	nh_.param("l_axis_trigger", l_trigger_, 2);
  nh_.param("r_axis_trigger", r_trigger_, 5);
	nh_.param("r_axis_linear", r_linear_, 4);
  nh_.param("r_axis_angular", r_angular_, 3);
  nh_.param("d_axis_up_down", d_axis_up_down_, 7);
	nh_.param("d_axis_left_right", d_axis_left_right_, 6);

	nh_.param("button_a", button_a_, 0);
  nh_.param("button_b", button_b_, 1);
	nh_.param("button_x", button_x_, 2);
  nh_.param("button_y", button_y_, 3);
	nh_.param("button_start", button_start_, 7);
  nh_.param("button_back", button_back_, 6);
	nh_.param("button_l_bumper", button_l_bumper_, 4);
  nh_.param("button_r_bumper", button_r_bumper_, 5);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	brake_pub_ = nh_.advertise<std_msgs::Bool>("/magellan_pro/cmd_brake_power", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &GodotJoystick::joyCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &GodotJoystick::odomCallback, this);

  next_linear_ = 0;
  next_angular_ = 0;
  count_ = 0;
  loop_rate_ = 100;
  turn_time_ = 1;
  angular_goal_ = 0;
  
  turning_ = false;
  
  ros::Rate loop_rate(loop_rate_);

  while(ros::ok())
  {
    //safety for bad maths
    if (next_angular_ > MAX_ANGULAR)
    {
      next_angular_ = MAX_ANGULAR;
      ROS_WARN("Max angular speed exceeded");
    }
    if (next_linear_ > MAX_LINEAR)
    {
      next_linear_ = MAX_LINEAR;
      ROS_WARN("Max linear speed exceeded");
    } 
    geometry_msgs::Twist cmd;
  	cmd.linear.x = next_linear_;
  	cmd.angular.z = next_angular_;
  	vel_pub_.publish(cmd);
  	ROS_DEBUG("published linear = %f,  angular = %f", next_linear_, next_angular_);
    count_++;
    ros::spinOnce(); 
    loop_rate.sleep();
  }

}

void GodotJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	ROS_DEBUG("Joystick Callback");
	
	if (joy->buttons[button_b_] == 1)
	{
		std_msgs::Bool brake;
		brake.data = true;
		brake_pub_.publish(brake);
	}
		
	if (joy->axes[l_trigger_] <= 0 && joy->axes[r_trigger_] <= 0) 
	{
    /* Use start button to turn off breaks */
		if (joy->buttons[button_start_] == 1)
		{
			std_msgs::Bool brake;
			brake.data = false;
			brake_pub_.publish(brake);
		}
		
		if (joy->buttons[button_x_] == 1)
	  {
	    turnOdom(true, 1.57);
	  }
	  else if (joy->buttons[button_a_] == 1)
	  {
	    turnOdom(false, 1.57);
	  }
	  else if (turning_ == false)
	  {
	    ROS_DEBUG("linear scale: %f, angular scale: %f", l_scale_, a_scale_);    

    	next_linear_ = l_scale_*joy->axes[l_linear_];
		  next_angular_ = a_scale_*joy->axes[l_angular_];
		
		  ROS_DEBUG("linear = %f,  angular = %f", next_linear_, next_angular_);
    }
    
  } else {
	  //Todo: make sure it stops if you release triggers while giving a command
	  //next_linear_ = 0;
	  //next_angular_ = 0;
  }
	
  /* Testing settings */
   /* if (joy->buttons[button_a_] == 1)
    {
			publish_rate += 1;
			ROS_INFO("Current publish rate is %d", publish_rate);
		} 
		else if (joy->buttons[button_x_] == 1)
    {
      publish_rate -= 1;
      ROS_INFO("Current publish rate is %d", publish_rate);
    }*/
}

void GodotJoystick::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  if (turning_ == true)
  {
    double angle = odom->pose.pose.orientation.z;
    next_angular_ = MAX_ANGULAR - (MAX_ANGULAR/(angular_goal_ - angle));
    
    ROS_INFO("Angle: %f, next_angular_: %f", angle, next_angular_);
    
    if (angle <= angular_goal_ + ANGULAR_THRESH && angle >= angular_goal_ - ANGULAR_THRESH) //almost certainly never true
      turning_ = false;
  }
}

bool GodotJoystick::turnOdom(bool clockwise, double radians)
{
  while(radians < 0) radians += 2*M_PI;
  while(radians > 2*M_PI) radians -= 2*M_PI;

  //wait for the listener to get the first message
  listener_.waitForTransform("base_link", "odom", 
                             ros::Time(0), ros::Duration(1.0));
  
  //we will record transforms here
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  listener_.lookupTransform("base_link", "odom", 
                            ros::Time(0), start_transform);
  
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to turn at 0.25 rad/s
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = 0.25;
  if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
  
  //the axis we want to be rotating by
  tf::Vector3 desired_turn_axis(0,0,1);
  if (!clockwise) desired_turn_axis = -desired_turn_axis;
  
  ros::Rate rate(10.0);
  bool done = false;
  while (!done && nh_.ok())
  {
    //send the drive command
    vel_pub_.publish(base_cmd);
    rate.sleep();
    //get the current transform
    try
    {
      listener_.lookupTransform("base_link", "odom", 
                                ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      break;
    }
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    tf::Vector3 actual_turn_axis = 
      relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    if ( fabs(angle_turned) < 1.0e-2) continue;

    if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
      angle_turned = 2 * M_PI - angle_turned;

    if (angle_turned > radians) done = true;
  }
  if (done) return true;
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "godot_joystick_node");
  GodotJoystick godot_joystick_node;

  //ros::spin();
}
