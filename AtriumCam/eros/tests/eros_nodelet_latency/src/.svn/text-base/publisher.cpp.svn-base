/**
 * @file /eros_nodelet_latency/src/publisher.cpp
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
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include "../msg_gen/cpp/include/eros_nodelet_latency/BigObject.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace eros_nodelet_latency {

/*****************************************************************************
** Class
*****************************************************************************/

class Publisher : public nodelet::Nodelet {
public:
    Publisher() : pub_thread(NULL), object(new BigObject()) {}
    ~Publisher() {
	    if ( pub_thread != NULL ) {
		    delete pub_thread;
	    }
	}

private:
    void onInit() {
		ros::NodeHandle& private_nh = getPrivateNodeHandle();
		pub = private_nh.advertise<BigObject>("/big_object", 10);
		object->data.assign(1000000, 1.1); // ~4MB
		pub_thread = new boost::thread(boost::bind(&Publisher::run,this));
    }

    void run() {
    	ros::WallTime time;
    	ros::Duration duration(0,500000000);
    	while (1) {
    		time = ros::WallTime::now();
    		object->stamp.sec = time.sec;
    		object->stamp.nsec = time.nsec;
    		pub.publish(object);
    		duration.sleep();
    	}
	}

	ros::Publisher pub;
	boost::thread *pub_thread;
	BigObjectPtr object;
};

PLUGINLIB_DECLARE_CLASS(eros_nodelet_latency, Publisher, eros_nodelet_latency::Publisher, nodelet::Nodelet);

} // namespace eros_nodelet_latency
