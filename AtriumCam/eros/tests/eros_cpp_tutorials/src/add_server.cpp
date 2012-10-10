/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "common.h"
#include "eros_cpp_tutorials/TwoInts.h"

/*****************************************************************************
** Main
*****************************************************************************/

bool add(eros_cpp_tutorials::TwoInts::Request  &req,
         eros_cpp_tutorials::TwoInts::Response &res ) {
	res.sum = req.a + req.b;
	ROS_INFO_STREAM(req.a << " + " << req.b << " = " << res.sum);
	return true;
}

int main(int argc, char **argv) {
//	tests::set_debug_log_levels();
	tests::init(argc, argv, "add_two_ints_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("add_two_ints", add);
	ros::spin();
	return 0;
}
