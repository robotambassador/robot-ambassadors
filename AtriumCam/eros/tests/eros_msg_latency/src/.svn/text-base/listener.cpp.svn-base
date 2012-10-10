/**
 * @file /eros_msg_latency/src/listener.cpp
 *
 * @brief Passing timestamped messages between processes.
 *
 * Listens to messages coming cross an ros topic with timestamps.
 *
 * Use with talker.cpp.
 *
 * @date August 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>

/*****************************************************************************
** Callbacks
*****************************************************************************/
/**
 * @brief The interprocess callback and average latency computer.
 */
class InterProcessListener {
public:
	InterProcessListener() : total_latency(0.0), count(0) {}
	/**
	 * @brief Catches messages coming from talker.
	 *
	 * This will be coming across a connection between two separate processes, so
	 * this will be about as slow as it gets for msg passing on localhost.
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
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	InterProcessListener listener;
	ros::Subscriber chatter_sub = n.subscribe("chatter", 100, &InterProcessListener::chatterCallback, &listener);
	while ( ros::ok() ) {
		ros::spin();
	}
	ROS_INFO("Avg Latency [%u]: %lf", listener.count, listener.averageLatency());

}

