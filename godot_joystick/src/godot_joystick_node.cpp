#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

static const double MAX_ANGULAR = 0.5; // rad/s
static const double MAX_LINEAR = 0.5; // m/s
static const double ANGULAR_THRESH = 5; 

class GodotJoystick
{
public:
  GodotJoystick();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;
  
  int l_linear_, l_angular_, r_linear_, r_angular_, l_trigger_, r_trigger_;

  int button_a_, button_b_, button_x_, button_y_, button_start_, button_back_, button_l_bumper_, button_r_bumper_;

  int d_axis_up_down_, d_axis_left_right_;

  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
	ros::Publisher brake_pub_;
  ros::Subscriber joy_sub_;
  
  
  double next_linear_;
  double next_angular_;
  int turn_angle_;
  int count_, loop_rate_, turn_time_;
  double angular_goal_, turn_speed_;
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

  next_linear_ = 0;
  next_angular_ = 0;
  count_ = 0;
  loop_rate_ = 100;
  turn_time_ = 100;
  angular_goal_ = 0;
  turn_speed_= 0.25;
  
  turning_ = false;
  
  ros::Rate loop_rate(loop_rate_);

  while(ros::ok())
  {
    if (turning_ == true)
    {
      count_++;
      if (count_ >= turn_time_)
	    {
	      next_angular_ = 0;
	      count_ = 0;
	      turning_ = false;	
	    }
    }
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
	    next_angular_ = turn_speed_;
	    count_ = 0;
	    turning_ = true;
	  }
	  else if (joy->buttons[button_a_] == 1)
	  {
	    next_angular_ = -turn_speed_;
	    count_ = 0;
	    turning_ = true;
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
	
  if (joy->buttons[button_l_bumper_] == 1)
  {
		turn_speed_ += 0.01;
		ROS_INFO("Current turn speed is %f", turn_speed_);
	} 
	else if (joy->buttons[button_r_bumper_] == 1)
  {
    turn_speed_ -= 0.01;
    ROS_INFO("Current turn speed is %f", turn_speed_);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "godot_joystick_node");
  GodotJoystick godot_joystick_node;

  //ros::spin();
}

