/**
 * @file /eros_msg_latency/src/thread_passing_copies.cpp
 *
 * @brief Passing copies internally between threads.
 *
 * @date August 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>

/*****************************************************************************
** Callbacks
*****************************************************************************/
/**
 * @brief The intraprocess callback and average latency computer for copy passing.
 */
class IntraCopyListener {
public:
	IntraCopyListener() : total_latency(0.0), count(0) {}
	/**
	 * @brief Catches messages coming from talker.
	 *
	 * This will be coming from another thread, but as a copy. Much faster than
	 * inter process, but can be slow for big objects.
	 *
	 * @param msg : incoming timestamped message.
	 */
	void chatterCallback(const std_msgs::Float64ConstPtr& msg) {
		++count;
		double latency = ros::WallTime::now().toSec() - msg->data;
		total_latency += latency;
		ROS_INFO("Latency: %f", latency);
	}

	/**
	 * @brief Calculate the current average latency.
	 *
	 * @return double : the calculated latency.
	 */
	double averageLatency() { return total_latency/static_cast<double>(count); }

	double total_latency;
	unsigned int count;

};

void chatterCallback(const std_msgs::Float64ConstPtr& msg) {
	double latency = ros::WallTime::now().toSec() - msg->data;
	ROS_INFO("Latency [%f]", latency);
}

/*****************************************************************************
** Listening Thread Function
*****************************************************************************/

void listener() {
	ros::NodeHandle n;
	IntraCopyListener copy_listener;
	ros::Subscriber chatter_sub = n.subscribe("chatter", 100, &IntraCopyListener::chatterCallback, &copy_listener);
	while ( ros::ok() ) {
		ros::spin();
	}
	ROS_INFO("Avg Latency [%u]: %lf", copy_listener.count, copy_listener.averageLatency());
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	// Note -> we are passing objects here, not pointers!
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("chatter", 100);
	ros::Rate loop_rate(5);
	int count = 0;
	boost::thread listener_thread(listener);
	while (ros::ok()) {
		std_msgs::Float64 timestamp;
		timestamp.data = ros::WallTime::now().toSec();
		chatter_pub.publish(timestamp);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	listener_thread.join();
}
