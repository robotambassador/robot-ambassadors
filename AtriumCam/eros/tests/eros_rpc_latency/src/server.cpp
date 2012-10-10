/**
 * @file /eros_rpc_latency/src/server.cpp
 *
 * @brief RPC latency benchmarking server.
 *
 * @date August 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "eros_rpc_latency/ping_pong.h"

/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @brief Benchmarking server node - runs the rpc_latency_benchmark service.
 *
 * When a service request is received, it timestamps the message and returns it.
 */
class LatencyServer {
public:
	/**
	 * Constructor - sets up the service.
	 */
	LatencyServer() :
		service (node.advertiseService("eros_rpc_latency", &LatencyServer::ping_pong, this))
	{}

	/**
	 * Nothing to do in here, just spin.
	 */
	void spin() {
		ros::spin();
	}
    /**
     * The rpc latency request callback. It timestamps it and returns it.
     */
	bool ping_pong(eros_rpc_latency::ping_pong::Request  &request,
			eros_rpc_latency::ping_pong::Response &response ) {
		timestamp = ros::WallTime::now();
		ros::Duration send_time(timestamp.sec - request.ping_sec,
				                timestamp.nsec - request.ping_nsec );
//		ROS_INFO("Send Time: %lf", send_time.toSec());
//		ROS_INFO("TimeStamp: %lf", timestamp.toSec());
		timestamp = ros::WallTime::now();
		response.pong_sec = timestamp.sec;
		response.pong_nsec = timestamp.nsec;
		return true;
	}

private:
	ros::WallTime timestamp;
	ros::NodeHandle node;
	ros::ServiceServer service;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
	ros::init(argc, argv, "eros_rpc_latency_server");

	LatencyServer latency_server;
	ROS_INFO("eros_rpc_latency_server waiting for requests...");
	latency_server.spin();


	return 0;
}


