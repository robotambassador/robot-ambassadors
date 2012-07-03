/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef FLYER_CONTROLLER_CONTROL_MODE_H
#define FLYER_CONTROLLER_CONTROL_MODE_H
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <joy/Joy.h>
#include "flyer_controller/control_mode_status.h"
#include "flyer_controller/control_mode_output.h"
#include "flyer_controller/control_mode_cmd.h"
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <algorithm>

using std::min;
using std::max;

#define CHECK_PARAMETER(x,msg) {if(!(x)) {ROS_FATAL_STREAM(msg); ros::requestShutdown(); return;}}

namespace flyer_controller
{
namespace ControlModeTypes
{
enum ControlModeStates
{
  ERROR = 0, OFF = 1, IDLE = 2, STANDBY = 3, ACTIVE = 4
};
typedef ControlModeStates ControlModeState;
}


class ControlMode : public nodelet::Nodelet
{
protected:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Parameters
  double status_report_rate;
  double control_output_rate;

  // Publishers
  ros::Publisher control_mode_status_pub;
  ros::Publisher output_pub;
  // Subscribers
  ros::Subscriber control_mode_cmd_sub;
  ros::Subscriber joy_sub;
  ros::Subscriber state_sub;
  // Timers
  ros::Timer report_status_timer;
  ros::Timer output_controls_timer;
  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;
  // Member variables
  ControlModeTypes::ControlModeState state; // mode state (OFF, IDLE, etc) for reporting status
  std::string info; // info string for reporting status
  joy::Joy latest_joy;
  bool ready;
  nav_msgs::Odometry latest_state;
  bool got_first_state;
  bool seen_max_alt;
  bool seen_min_alt;
  bool got_first_joy;
  // Methods
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void requestRegistration(const std::string& mode_name, double wait_time=0);
  void requestRegistration2(const ros::TimerEvent& event, const std::string& mode_name, double wait_time=0);
  void startDataFlow();
  virtual void controlModeCmdCallback(const control_mode_cmdConstPtr& msg);
  virtual void joyCallback(const joy::JoyConstPtr& msg);
  virtual void stateCallback(const nav_msgs::OdometryConstPtr& msg);
  virtual void reportStatusTimerCallback(const ros::TimerEvent& e);
  virtual void outputControlTimerCallback(const ros::TimerEvent& e);
  // Members
  double latest_alt_cmd; // 0..1 (scaled and clipped)
  ros::Timer reg_timer;

public:
  ControlMode();
  ~ControlMode();
  std::string stateToString(ControlModeTypes::ControlModeState t);
protected:
  virtual void onInit();

};


}
#endif
