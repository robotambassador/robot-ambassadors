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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <diagnostic_updater/diagnostic_updater.h>
#include "flyer_controller/controller_status.h"
#include "flyer_controller/controller_cmd.h"
#include "std_msgs/Bool.h"

#include <joy/Joy.h>

namespace flyer_controller
{
// Button definitions
// TRIGGER
const int BTN_TRIGGER = 0;
const int BTN_PROCEED = 8;
const int BTN_CANCEL = 6;
const int BTN_ESTOP = 11;
// Standby Mode
const int BTN_STANDBY_GO_OPERATIONAL = 2;
// Operational Mode
const int BTN_OPERATIONAL_MODE_ATTITUDE = 2;
const int BTN_OPERATIONAL_MODE_HOVER = 3;

namespace TeleopTypes
{
enum TeleopStates
{
  ERROR = 0, OFF = 1, INITIALIZE = 2, STANDBY = 3, OPERATIONAL = 4
};
typedef TeleopStates TeleopState;

}

using namespace TeleopTypes;
using namespace std;

class TeleopFlyer : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Publishers
  ros::Publisher joy_repub;
  ros::Publisher controller_cmd_pub;
  ros::Publisher motor_enable_pub;
  ros::Publisher estop_pub;
  // Subscribers
  ros::Subscriber joy_sub;
  ros::Subscriber controller_status_sub;
  // Timer
  ros::Timer republish_timer;
  // Members
  bool got_first_joy;
  joy::Joy prev_joy;
  joy::Joy latest_joy;
  ros::Time latest_joy_time;

  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;

  // Parameters
  double joy_republish_rate; // [Hz] rate to republish Joy messages at
  double max_interval; // [s] max time between received Joy messages

  // Members
  TeleopState state;
  bool command_pending; // is there a command pending?
  string pending_command; // the pending command
  std_msgs::Bool mtr_enable_msg;
  bool lost_joystick; // Latches true if no joystic message seen for too long
  double last_trigger_duration;
  controller_status latest_controller_status;

public:
  TeleopFlyer() :
    got_first_joy(false), diag_updater(), joy_republish_rate(20), max_interval(0.5), state(OFF),
        command_pending(false), pending_command(""), lost_joystick(false), last_trigger_duration(0)
  {
  }

  void onInit()
  {
    nh = getMTNodeHandle();
    nh_priv = getMTPrivateNodeHandle();

    // Diagnostics
    diag_updater.add("TeleopFlyer Status", this, &TeleopFlyer::diagnostics);
    diag_updater.setHardwareID("none");
    diag_updater.force_update();
    // Parameters
    nh_priv.param("joy_republish_rate", joy_republish_rate, joy_republish_rate);
    nh_priv.param("max_interval", max_interval, max_interval);
    // Publishers
    joy_repub = nh_priv.advertise<joy::Joy> ("joy", 1);
    controller_cmd_pub = nh_priv.advertise<controller_cmd> ("controller_cmd", 1, true);
    motor_enable_pub = nh_priv.advertise<std_msgs::Bool> ("motor_enable", 1);
    estop_pub = nh_priv.advertise<std_msgs::Bool> ("estop", 1);
    // Subscribers
    joy_sub = nh.subscribe("joy", 10, &TeleopFlyer::joyCallback, this, ros::TransportHints().unreliable().tcpNoDelay());
    controller_status_sub = nh.subscribe("controller/status", 10, &TeleopFlyer::controllerStatusCallback, this,
                                         ros::TransportHints().unreliable().tcpNoDelay());
    // Timers
    republish_timer = nh.createTimer(ros::Duration(1 / joy_republish_rate), &TeleopFlyer::republishCallback, this);
  }

  std::string stateToString(TeleopState t)
  {
    switch (t)
    {
      case OFF:
        return "OFF";
      case ERROR:
        return "ERROR";
      case INITIALIZE:
        return "INITIALIZE";
      case STANDBY:
        return "STANDBY";
      case OPERATIONAL:
        return "OPERATIONAL";
    }
    return "Unknown";
  }

private:

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    int status;
    status = ((state != ERROR && !lost_joystick) ? int(diagnostic_msgs::DiagnosticStatus::OK)
        : int(diagnostic_msgs::DiagnosticStatus::ERROR));
    stat.summary(status, stateToString(state));
    stat.add("Command Pending", command_pending);
    stat.add("Pending Command", pending_command);
    stat.add("Lost Joystick", lost_joystick);
    stat.add("Trigger Duration", last_trigger_duration);
  }

  void controllerStatusCallback(const controller_statusConstPtr& msg)
  {
    state = TeleopState(msg->state); // make teleop's state match that of Controller
    latest_controller_status = *msg;
  }

  void estop()
  {
    static bool issued_warning = false;
    std_msgs::Bool estop_msg;
    estop_msg.data = true;
    estop_pub.publish(estop_msg);
    if (not issued_warning)
    {
      ROS_WARN("E-STOP triggered");
      issued_warning = true;
    }
  }

  void joyCallback(joy::Joy msg)
  {
    //ROS_DEBUG("Joy callback called");
    static ros::Time trigger_pull_time;
    ros::Time rightnow = ros::Time::now();
    //latest_joy = msg.get();
    prev_joy = latest_joy;
    latest_joy = msg;
    if (!got_first_joy)
    {
      got_first_joy = true;
      ROS_INFO("Got first joystick message");
    }
    else
    {
      double dT = (rightnow - latest_joy_time).toSec();
      if (dT > max_interval and (state == OPERATIONAL))
      {
        ROS_ERROR_STREAM("Time (" << dT << " s) between Joy messages exceeded allowable (" << max_interval << ")");
        lost_joystick = true;
      }

      // Special case for estop
      if (latest_joy.buttons[BTN_ESTOP] == 1)
      {
        estop();
      }

      // Special cases for trigger:
      // If trigger has just been pressed, record the current
      // time
      if (latest_joy.buttons[BTN_TRIGGER] == 1 and prev_joy.buttons[BTN_TRIGGER] == 0)
      {
        trigger_pull_time = rightnow;
        ROS_INFO("Trigger pressed");
      }
      // If button 0 transitions from 1 to 0 in
      // any mode, set motor_enable 'line' to false and immediately
      // publish the message
      else if (latest_joy.buttons[BTN_TRIGGER] == 0 and prev_joy.buttons[BTN_TRIGGER] == 1)
      {
        mtr_enable_msg.data = false;
        motor_enable_pub.publish(mtr_enable_msg);
        last_trigger_duration = (rightnow - trigger_pull_time).toSec();
        ROS_INFO_STREAM("Trigger released - duration was: " << last_trigger_duration);
      }

      // Check for a 'button-up' event; it may be a command
      for (unsigned int i = 0; i < latest_joy.buttons.size(); i++)
      {
        if (latest_joy.buttons[i] == 0 and prev_joy.buttons[i] == 1 and i != BTN_TRIGGER) // trigger can never be a command button
        {
          doCommand(i);
          continue;
        }
      }
    }
    latest_joy_time = rightnow;
  }

  void doCommand(int button_num)
  {
    ROS_DEBUG_STREAM("Button command, button #: " << button_num);
    if (command_pending)
    {
      if (button_num == BTN_CANCEL) // Cancel
      {
        ROS_WARN("Pending command canceled");
        command_pending = false;
        pending_command = "";
      }
      else if (button_num == BTN_PROCEED) // Proceed
      {
        //ROS_INFO_STREAM("Sending command: " << pending_command);
        controller_cmd cmd_msg;
        cmd_msg.cmd = pending_command;
        controller_cmd_pub.publish(cmd_msg);
        command_pending = false;
        pending_command = "";
      }
      else
      {
        ROS_WARN("Command pending - proceed or cancel");
      }
    }
    else
    {
      switch (state)
      {
        case STANDBY:
        {
          switch (button_num)
          {
            case BTN_STANDBY_GO_OPERATIONAL:
            {
              command_pending = true;
              pending_command = "mode operational";
            }
              break;
          }
        }
          break;
        case OPERATIONAL:
        {
          // Thoughts: in OPERATIONAL, we might have an arbitrary number of modes to switch between, but there will be some
          // basic modes (idle, attitude, hover, ...) that can/should have dedicated buttons. For the others we can reserve
          // a button or two to cycle through the other available modes.
          // Also: should there be a button that is like the 'Esc' key, that takes you back to the mode you were in before
          // the one you last switched to? Should the modes be pushed onto a stack??
          switch (button_num)
          {
            case BTN_OPERATIONAL_MODE_ATTITUDE:
            {
              command_pending = true;
              pending_command = "control_mode to_active attitude";
            }
              break;
            case BTN_OPERATIONAL_MODE_HOVER:
            {
              command_pending = true;
              pending_command = "control_mode to_active hover";
            }
              break;
          }
        }
          break;
        default:
        {
          ROS_WARN_STREAM("Unrecognized button: " << button_num);
        }
      }
      if (command_pending)
      {
        ROS_DEBUG_STREAM("Command pending: " << pending_command);
      }
    }
  }

  void republishCallback(const ros::TimerEvent& e)
  {
    //ROS_DEBUG_STREAM("Timer callback triggered " << e.current_real.toSec());
    if (got_first_joy)
    {
      joy::JoyPtr joy_msg_ptr(new joy::Joy);
      *joy_msg_ptr = latest_joy;
      joy_repub.publish(joy_msg_ptr);
    }
    diag_updater.update();
    ros::Time now = ros::Time::now();
    double joy_interval = (now - latest_joy_time).toSec();
    bool joystick_is_recent = (!got_first_joy) || (joy_interval < max_interval);
    if (lost_joystick)
    {
      estop();
      mtr_enable_msg.data = false;
      motor_enable_pub.publish(mtr_enable_msg);
    }
    else
    {
      if (joystick_is_recent)
      {
        if (state == OPERATIONAL)
        {
          mtr_enable_msg.data = (latest_joy.buttons[BTN_TRIGGER] == 1);
          motor_enable_pub.publish(mtr_enable_msg);
        }
        else
        {
          mtr_enable_msg.data = false;
          motor_enable_pub.publish(mtr_enable_msg);
        }
      }
      else
      {
        if (got_first_joy and (state == OPERATIONAL)) // can't lose it until we've first had it
        {
          lost_joystick = true;
          ROS_ERROR_STREAM("Too large (" << joy_interval << " s) an interval between joystick messages!");
          mtr_enable_msg.data = false;
          motor_enable_pub.publish(mtr_enable_msg);
        }
      }
    }
  }

};
PLUGINLIB_DECLARE_CLASS(flyer_controller, TeleopFlyer, flyer_controller::TeleopFlyer, nodelet::Nodelet)
;

} // namespace

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "teleop_flyer");
//  flyer_controller::TeleopFlyer teleop_flyer;
//
//  ros::spin();
//
//  return 0;
//}
