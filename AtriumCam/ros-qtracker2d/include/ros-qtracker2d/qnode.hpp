/**
 * @file /include/ros-qtracker2d/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef QNODE_HPP_
#define QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "ros-qtracker2d/processingthread.hpp"

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	//QNode(int argc, char** argv );
	QNode();
	~QNode();
	bool init(ProcessingThread* processingThread);
	//bool init(const std::string &master_url, const std::string &host_url, const ProcessingThread::ProcessingThread &pThread);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

signals:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher location_publisher;
    	QStringListModel logging_model;
	ProcessingThread* pThread;
};

#endif /* QNODE_HPP_ */
