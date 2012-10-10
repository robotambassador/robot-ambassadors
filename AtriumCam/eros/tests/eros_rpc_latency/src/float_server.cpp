/**
 * @file /eros_rpc_latency/src/float_server.cpp
 *
 * @brief RPC latency benchmarking server with floats.
 *
 * @date August 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "eros_rpc_latency/float_ping_pong.h"

/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @brief Benchmarking server node for float type rpc requests/responses.
 *
 * When a service request is received, it timestamps the message and returns it.
 */
class FloatLatencyServer {
public:
	/**
	 * Constructor - sets up the service.
	 */
	FloatLatencyServer() :
		service (node.advertiseService("eros_rpc_latency", &FloatLatencyServer::ping_pong, this))
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
	bool ping_pong(eros_rpc_latency::float_ping_pong::Request  &request,
			eros_rpc_latency::float_ping_pong::Response &response ) {
		timestamp = ros::WallTime::now();
		ros::Duration send_time(timestamp.toSec() - request.ping_time);
//		ROS_INFO("Send Time: %lf", send_time.toSec());
//		ROS_INFO("TimeStamp: %lf", timestamp.toSec() );
		response.pong_time = ros::WallTime::now().toSec();
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

	FloatLatencyServer latency_server;
	ROS_INFO("eros_rpc_latency_server waiting for requests...");
	latency_server.spin();


	return 0;
}


