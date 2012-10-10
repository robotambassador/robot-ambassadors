/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include "ros-qtracker2d/qnode.hpp"
#include "ros-qtracker2d/processingthread.hpp"

/*****************************************************************************
** Implementation
*****************************************************************************/

//QNode::QNode(int argc, char** argv ) :
//	init_argc(argc),
//	init_argv(argv)
QNode::QNode()
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init(ProcessingThread* processingThread) {
	ros::init(init_argc,init_argv,"rosqtracker");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	pThread = processingThread;
	// Add your ros communications here.
	location_publisher = n.advertise<geometry_msgs::Pose>("robot_location", 1000);
	start();
}

/*bool QNode::init(const std::string &master_url, const std::string &host_url, const ProcessingThread::ProcessingThread &pThread) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"test");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	location_publisher = n.advertise<std_msgs::String>("robot_location", 1000);
	start();
}*/

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		CvPoint robotLocation = pThread->robotLocation;
		geometry_msgs::Pose location;
		if (robotLocation.x > 1000) {
			location.position.x = 0.0;
		} else {
			location.position.x = (robotLocation.x/1.45)/100;
		}
		if (robotLocation.y > 1000) {
                        location.position.y = 0.0;
                } else {
			location.position.y =  (robotLocation.y/1.45)/100;
		}

		location_publisher.publish(location);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	emit loggingUpdated(); // used to readjust the scrollbar
}
