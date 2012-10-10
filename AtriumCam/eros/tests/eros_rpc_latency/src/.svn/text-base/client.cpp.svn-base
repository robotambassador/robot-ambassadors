/**
 * @file /eros_rpc_latency/src/client.cpp
 *
 * @brief RPC latency benchmarking client.
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
 * @brief Benchmarking client node for integral type rpc requests/responses.
 *
 * Effectively sends a timestamped request (using ints as the storage type) and
 * waits for the reply. When it receives the reply, it checks the server's timestamp and
 * then calculates one-way and round-trip latencies before printing them.
 *
 * Once completed, the node shuts down and prints average outgoing (request) and incoming
 * (response) latencies.
 */
class LatencyClient {
public:
	LatencyClient() :
		client(node.serviceClient< eros_rpc_latency::ping_pong >("eros_rpc_latency")),
		total_request(0.0),
		total_response(0.0),
		cnt(0)
	{}
	/**
	 * Latency benchmarking service request handler. This initiates the request and waits
	 * for the response.
	 * @return bool - success or failure of the request.
	 */
	bool ping_pong() {
		eros_rpc_latency::ping_pong srv;
		ros::WallTime send_stamp, final_stamp;
		ros::Duration send_duration, receive_duration;
		send_stamp = ros::WallTime::now();

		srv.request.ping_sec = send_stamp.sec;
		srv.request.ping_nsec = send_stamp.nsec;
		if ( client.call(srv) ) {
			final_stamp = ros::WallTime::now();
			send_duration.sec = srv.response.pong_sec - srv.request.ping_sec;
			send_duration.nsec = srv.response.pong_nsec - srv.request.ping_nsec;
			receive_duration.sec = final_stamp.sec - srv.response.pong_sec;
			receive_duration.nsec = final_stamp.nsec - srv.response.pong_nsec;
			ROS_INFO("Latency [request,response] : [%lf,%lf]", send_duration.toSec(), receive_duration.toSec());
			total_request += send_duration.toSec();
			total_response += receive_duration.toSec();
			++cnt;
			return true;
		} else {
			if ( cnt == 0 ) {
				ROS_ERROR("Failed to call service eros_rpc_latency");
			} // else it's probably just terminating from a ctrl-c.
			return false;
		}
	}
	double averageRequestLatency() { return (total_request/static_cast<double>(cnt)); }
	double averageResponseLatency() { return (total_response/static_cast<double>(cnt)); }
	unsigned int count() { return cnt; }

private:
	ros::NodeHandle node;
	ros::ServiceClient client;
	double total_request, total_response;
	unsigned int cnt;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
	ros::init(argc, argv, "eros_rpc_latency_client");

	LatencyClient latency;
	while( ros::ok() ) {
		latency.ping_pong();
		sleep(1);
	}
	ROS_INFO("Avg Latency [request,response] [%lf,%lf] [%u]", latency.averageRequestLatency(), latency.averageResponseLatency(), latency.count());
	return 0;
}


