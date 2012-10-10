/**
 * @file /eros_cpp_tutorials/src/add_client.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Mar 1, 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdlib>
#include <ros/ros.h>
#include "eros_cpp_tutorials/TwoInts.h"
#include "common.h"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
//	tests::set_debug_log_levels();
	tests::init(argc, argv, "add_two_ints_client");
	ros::NodeHandle n;
	ros::ServiceClient add_client = n.serviceClient<eros_cpp_tutorials::TwoInts>("add_two_ints");
	ros::Rate loop_rate(1);

	int last_sum = 0;
	int count = 1;
	while( ros::ok() ) {
		eros_cpp_tutorials::TwoInts srv;
		srv.request.a = count;
		srv.request.b = last_sum;
		if ( add_client.call(srv) ) {
			ROS_INFO_STREAM(srv.request.a << " + " << srv.request.b << " = " << srv.response.sum);
		} else {
			ROS_ERROR("Failed to call service add_two_ints");
			break;
		}
		last_sum = srv.response.sum;
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
