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
#include "flyer_controller/control_mode.h" // This class' header
#include "flyer_controller/control_modes.h" // srv
//using namespace flyer_controller;
namespace flyer_controller
{

std::string ControlMode::stateToString(ControlModeTypes::ControlModeState t)
{
  switch (t)
  {
    case ControlModeTypes::OFF:
      return "OFF";
    case ControlModeTypes::ERROR:
      return "ERROR";
    case ControlModeTypes::IDLE:
      return "IDLE";
    case ControlModeTypes::STANDBY:
      return "STANDBY";
    case ControlModeTypes::ACTIVE:
      return "ACTIVE";
  }
  return "Unknown";
}

void ControlMode::onInit()
{
  NODELET_INFO("ControlMode onInit() called");
  nh = getMTNodeHandle();
  nh_priv = getMTPrivateNodeHandle();

  // Diagnostics
  diag_updater.add(getName() + " status", this, &ControlMode::diagnostics);
  diag_updater.setHardwareID("none");
  diag_updater.force_update();

  // Parameters
  nh_priv.param("status_report_rate", status_report_rate, status_report_rate);
  nh_priv.param("control_output_rate", control_output_rate, control_output_rate);
  CHECK_PARAMETER((status_report_rate > 0), "parameter value out of range");
  CHECK_PARAMETER((control_output_rate > 0), "parameter value out of range");

  // Publishers
  control_mode_status_pub = nh_priv.advertise<control_mode_status> ("status", 1, true);
}

ControlMode::ControlMode() :
  status_report_rate(5), control_output_rate(10), state(ControlModeTypes::OFF), ready(false), got_first_state(false),
      seen_max_alt(false), seen_min_alt(false), got_first_joy(false), latest_alt_cmd(0)
{
}

///\brief Publishes diagnostics and status
void ControlMode::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  int status;
  status = ((state != ControlModeTypes::ERROR) ? diagnostic_msgs::DiagnosticStatus::OK
      : diagnostic_msgs::DiagnosticStatus::ERROR);
  stat.summary(status, stateToString(state));
  stat.add("ready", ready);
  stat.add("info", info);
}

void ControlMode::requestRegistration(const std::string& mode_name, double wait_time)
{
  reg_timer = nh.createTimer(ros::Duration(0.01), boost::bind(&ControlMode::requestRegistration2, this, _1, mode_name,
                                                              wait_time), true);
}

void ControlMode::requestRegistration2(const ros::TimerEvent& event, const std::string& mode_name, double wait_time)
{
  // Service Client
  NODELET_INFO("Waiting %f seconds before trying to register..", wait_time);
  ros::Duration d(wait_time);
  d.sleep();
  NODELET_INFO("Waiting for controller/control_modes service...");
  info = "Waiting for controller/control_mode service...";
  ros::service::waitForService("controller/control_modes"); // blocks until service available, no timeout
  info = "registering service...";
  control_modes register_service;
  register_service.request.request = "register " + mode_name + " " + getName(); //ros::this_node::getName();
  bool registered = ros::service::call("controller/control_modes", register_service);
  if (registered)
  {
    if (register_service.response.response == "success")
    {
      //NODELET_INFO("Successfully registered control mode");
      info = "control mode registered";
      startDataFlow();
    }
    else
    {
      NODELET_ERROR_STREAM ("Control mode registration failed, response: '" << register_service.response.response
          << "', reason: '" << register_service.response.reason << "'");
      state = ControlModeTypes::ERROR;
      info = "registration failed";
    }
  }
  else
  {
    NODELET_ERROR("Control mode registration service call failed");
    state = ControlModeTypes::ERROR;
    info = "registration service call failed";
  }
}

ControlMode::~ControlMode()
{
  NODELET_INFO("Destructor called");
}

void ControlMode::startDataFlow()
{
  //NODELET_INFO("Setting up publishers, subscribers, and timers..");
  // Publishers
  output_pub = nh_priv.advertise<control_mode_output> ("output", 1);
  // Subscribers
  control_mode_cmd_sub = nh_priv.subscribe("cmd", 10, &ControlMode::controlModeCmdCallback, this);
  joy_sub = nh.subscribe("teleop_flyer/joy", 10, &ControlMode::joyCallback, this);
  state_sub = nh.subscribe("odom", 10, &ControlMode::stateCallback, this);
  // Timers
  output_controls_timer = nh.createTimer(ros::Duration(1 / control_output_rate),
                                         &ControlMode::outputControlTimerCallback, this);
  report_status_timer = nh.createTimer(ros::Duration(1 / status_report_rate), &ControlMode::reportStatusTimerCallback,
                                       this);

  state = ControlModeTypes::IDLE;
  info = "data flow started";

}

void ControlMode::controlModeCmdCallback(const control_mode_cmdConstPtr& msg)
{
  NODELET_INFO_STREAM("Heard command: " << msg->cmd);
  if (msg->cmd == "mode idle")
  {
    state = ControlModeTypes::IDLE;
    info = "";
  }
  else if (msg->cmd == "mode standby")
  {
    state = ControlModeTypes::STANDBY;
    info = "";
  }
  else if (msg->cmd == "mode active")
  {
    if (state == ControlModeTypes::STANDBY)
    {
      state = ControlModeTypes::ACTIVE;
      info = "";
    }
    else
    {
      NODELET_ERROR("Invalid transition");
    }
  }
  else
  {
    NODELET_ERROR_STREAM("Command unknown: " << msg->cmd);
  }
}

void ControlMode::joyCallback(const joy::JoyConstPtr& msg)
{
  //NODELET_DEBUG_STREAM(__PRETTY_FUNCTION__ << " callback called");
  latest_joy = *msg;
  if (!got_first_joy)
    got_first_joy = true;
  //NODELET_INFO_STREAM("got_first_joy = " << got_first_joy);
  if (!seen_max_alt)
  {
    seen_max_alt = (msg->axes[3] >= 0.95);
  }
  else
  {
    if (!seen_min_alt)
    {
      seen_min_alt = (msg->axes[3] <= -0.95);
    }
  }
}

void ControlMode::stateCallback(const nav_msgs::OdometryConstPtr& msg)
{
  //NODELET_DEBUG_STREAM(__PRETTY_FUNCTION__ << " callback called");
  if (!got_first_state)
    got_first_state = true;
  latest_state = *msg;
  //NODELET_INFO_STREAM("Got state, z = " << latest_state.pose.pose.position.z);
}

void ControlMode::reportStatusTimerCallback(const ros::TimerEvent& e)
{
  control_mode_statusPtr msg(new control_mode_status);
  //NODELET_INFO_STREAM(__PRETTY_FUNCTION__);
  msg->state = state;
  msg->info = info;
  control_mode_status_pub.publish(msg);
  diag_updater.update();
}

void ControlMode::outputControlTimerCallback(const ros::TimerEvent& e)
{
  //NODELET_INFO_STREAM(__PRETTY_FUNCTION__);
  control_mode_outputPtr control_out(new control_mode_output);
  switch (state)
  {
    case ControlModeTypes::STANDBY:
    case ControlModeTypes::ACTIVE:
      output_pub.publish(control_out);
      break;
    default:
      break;
  }
}


}// namespace

