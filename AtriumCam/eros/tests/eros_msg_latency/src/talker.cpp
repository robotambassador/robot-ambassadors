/**
 * @file /eros_msg_latency/src/talker.cpp
 *
 * @brief Passing timestamped messages between processes.
 *
 * Use with client.cpp.
 *
 * @date August 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("chatter", 100);
	ros::Rate loop_rate(1);
	int count = 0;
	while (ros::ok()) {
		std_msgs::Float64 timestamp;
		timestamp.data = ros::WallTime::now().toSec();
		chatter_pub.publish(timestamp);
		ROS_INFO("I published [%f]", timestamp.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
}
