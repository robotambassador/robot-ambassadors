#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

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
  
  
  double next_linear;
  double next_angular;
  
};


GodotJoystick::GodotJoystick()
{

  nh_.param("scale_angular", a_scale_, -0.6);
  nh_.param("scale_linear", l_scale_, 0.6);

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

  next_linear = 0;
  next_angular = 0;
  
  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    
    geometry_msgs::Twist cmd;
  	cmd.linear.x = next_linear;
  	cmd.angular.z = next_angular;
  	vel_pub_.publish(cmd);
  	ROS_DEBUG("published linear = %f,  angular = %f", next_linear, next_angular);
    
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

    ROS_DEBUG("linear scale: %f, angular scale: %f", l_scale_, a_scale_);    

  	next_linear = l_scale_*joy->axes[l_linear_];
		next_angular = a_scale_*joy->axes[l_angular_];
		
		ROS_DEBUG("linear = %f,  angular = %f", next_linear, next_angular);

	} else {
		//Todo: make sure it stops if you release triggers while giving a command
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "godot_joystick_node");
  GodotJoystick godot_joystick_node;

  //ros::spin();
}
