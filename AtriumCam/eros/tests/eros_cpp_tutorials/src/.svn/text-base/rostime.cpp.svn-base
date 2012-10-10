/**
 * @file /src/rostime.cpp
 *
 * @brief Test the rostime now() and sleep() implementations.
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/time.h>
#include <iostream>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	ros::Time::init();
	ros::Time timestamp = ros::Time::now();
	std::cout << "Time: " << timestamp << std::endl;

	std::cout << "Sleeping 3s" << std::endl;
	ros::Duration three_secs(3,0);
	three_secs.sleep();
	timestamp = ros::Time::now();
	std::cout << "Time: " << timestamp << std::endl;
	std::cout << "Sleeping 1ms" << std::endl;
	ros::Duration millisecond(0,1000000);
	millisecond.sleep();
	timestamp = ros::Time::now();
	std::cout << "Time: " << timestamp << std::endl;
	return 0;
}

