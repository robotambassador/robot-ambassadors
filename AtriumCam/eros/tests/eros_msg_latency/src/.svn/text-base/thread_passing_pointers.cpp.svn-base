/**
 * @file /eros_msg_latency/src/thread_passing_pointers.cpp
 *
 * @brief Passing pointers internally between threads.
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
 * @brief The intraprocess callback and average latency computer for pointer passing.
 */
class IntraPointerListener {
public:
	IntraPointerListener() : total_latency(0.0), count(0) {}
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


/*****************************************************************************
** Listening Thread Function
*****************************************************************************/

void listener() {
	ros::NodeHandle n;
	IntraPointerListener ptr_listener;
	ros::Subscriber chatter_sub = n.subscribe("chatter", 100, &IntraPointerListener::chatterCallback, &ptr_listener);
	while ( ros::ok() ) {
		ros::spin();
	}
	ROS_INFO("Avg Latency [%u]: %lf", ptr_listener.count, ptr_listener.averageLatency());
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("chatter", 100);
	ros::Rate loop_rate(5);
	int count = 0;
	boost::thread listener_thread(listener);
	std_msgs::Float64Ptr timestamp(new std_msgs::Float64);
	while (ros::ok()) {
		timestamp->data = ros::WallTime::now().toSec();
		chatter_pub.publish(timestamp); // <---- note we pass a shared pointer here, not an object itself!
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	listener_thread.join();
}
