/**
 * @file /eros_nodelet_latency/src/subscriber.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 12/08/2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "../msg_gen/cpp/include/eros_nodelet_latency/BigObject.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace eros_nodelet_latency {

/*****************************************************************************
** Class
*****************************************************************************/

class Subscriber : public nodelet::Nodelet {
public:
    Subscriber() : count(0), sum_latency(0,0) {}

private:
    void onInit() {
		ros::NodeHandle& nh = getPrivateNodeHandle();
		sub = nh.subscribe(std::string("/big_object"), 10, &Subscriber::callback, this);
    }

    void callback(const BigObjectPtr object) {
		++count;
		ros::WallTime time = ros::WallTime::now();
		ros::Duration latency(time.sec - object->stamp.sec, time.nsec - object->stamp.nsec);
		sum_latency += latency;
		std::cout << "Avg Latency [" << count << "]: " << sum_latency.toSec()/static_cast<double>(count)  << std::endl;
	}

	ros::Subscriber sub;
	unsigned int count;
    ros::Duration sum_latency;
};

PLUGINLIB_DECLARE_CLASS(eros_nodelet_latency, Subscriber, eros_nodelet_latency::Subscriber, nodelet::Nodelet);

} // namespace eros_nodelet_latency
