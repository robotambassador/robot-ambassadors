
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "common.h"
#include <std_msgs/String.h>

/*****************************************************************************
** Main
*****************************************************************************/

/**
 * Just checks that xmlrpc comms can connect and talk with the master.
 */
int main(int argc, char **argv)
{
	tests::init(argc, argv, "nodelist");
	tests::set_debug_log_levels();
	ros::Time::init();

	ros::V_string nodes;
	if ( ros::master::getNodes(nodes) ) {
		std::cout << "Node list:" << std::endl;
		for (unsigned int i = 0; i < nodes.size(); ++i ) {
			std::cout << "  " << nodes[i] << std::endl;
		}
	} else {
		ROS_ERROR("Couldn't contact the master");
	}
  return 0;
}
