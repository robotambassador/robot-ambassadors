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
#include "flyer_controller/joystick_modes.h"
#include "flyer_controller/attitude_mode.h"
#include <math.h>
#include <algorithm>
#include <tf/transform_datatypes.h>

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)

using namespace flyer_controller;
using namespace std;
using namespace ControlModeTypes;

namespace flyer_controller
{
class ControlModeAttitude : public JoystickMode
{
private:
  // Parameters

  // Members
  AttitudeController att_control;

public:
  ControlModeAttitude() :
    att_control()
  {

  }

  void onInit()
  {
    JoystickMode::onInit();
    // Parameters
    double max_pitch_cmd = 10; // [deg] commanded pitch corresponding to full deflection
    double max_roll_cmd = 10; // [deg] commanded roll corresponding to full deflection
    bool external_command_frame = false; // should commands be interpreted w.r.t. an external frame?
    double external_frame_heading = 0; // [deg] heading that will correspond to a -pitch command when using external frame
    bool direct_yaw_rate_control = false; // joystick yaw rate commands sent directly

    nh_priv.param("max_pitch_cmd", max_pitch_cmd, max_pitch_cmd);
    nh_priv.param("max_roll_cmd", max_roll_cmd, max_roll_cmd);
    nh_priv.param("external_command_frame", external_command_frame, external_command_frame);
    nh_priv.param("external_frame_heading", external_frame_heading, external_frame_heading);
    nh_priv.param("direct_yaw_rate_control", direct_yaw_rate_control, direct_yaw_rate_control);
    CHECK_PARAMETER(max_pitch_cmd >= 0, "parameter value out of range");
    CHECK_PARAMETER(max_roll_cmd >= 0, "parameter value out of range");

    CHECK_PARAMETER((external_frame_heading >= -360) && (external_frame_heading <= 360), "parameter value out of range");
    NODELET_INFO_STREAM("max_pitch_cmd = " << max_pitch_cmd);

    att_control.configure(external_command_frame, external_frame_heading, direct_yaw_rate_control, max_pitch_cmd,
                          max_roll_cmd, max_yaw_rate_cmd, min_alt_cmd, max_alt_cmd);

    requestRegistration("attitude");
  }

private:
  // Virtual methods from base class
  void outputControlTimerCallback(const ros::TimerEvent& e)
  {
    control_mode_output control_out;
    bool do_calcs = false;
    bool do_publish = false;

    switch (state)
    {
      case ControlModeTypes::STANDBY:
        do_calcs = got_first_joy and got_first_state;
        do_publish = true;
        break;
      case ControlModeTypes::ACTIVE:
        do_calcs = true;
        do_publish = true;
        break;
      default:
        break;
    }

    if (do_calcs)
    {
      att_control.outputControl(boost::make_shared<nav_msgs::Odometry>(latest_state), latest_cmd,
                                boost::make_shared<joy::Joy>(latest_joy),
                                control_out);
    }

    if (do_publish)
    {
      control_out.control_mode = "attitude";
      output_pub.publish(control_out);
    }

  }

private:
  void reportStatusTimerCallback(const ros::TimerEvent& e)
  {
    control_mode_status msg;
    //NODELET_INFO_STREAM(__PRETTY_FUNCTION__);
    msg.state = state;
    msg.info = info;
    ready = (got_first_joy and got_first_state and seen_max_alt and seen_min_alt and latest_alt_cmd < 0.05);
    msg.ready = ready; // TODO: check some more conditions..
    control_mode_status_pub.publish(msg);
    diag_updater.update();
  }

  void controlModeCmdCallback(const control_mode_cmdConstPtr& msg)
  {
    NODELET_INFO_STREAM("Heard command: " << msg->cmd);
    if (msg->cmd == "mode idle")
    {
      state = IDLE;
      info = "";
    }
    else if (msg->cmd == "mode standby")
    {
      state = STANDBY;
      info = "";
    }
    else if (msg->cmd == "mode active")
    {
      if (state == STANDBY)
      {
        state = ACTIVE;
        att_control.useCurrentYaw(boost::make_shared<nav_msgs::Odometry>(latest_state));
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

}; // class
PLUGINLIB_DECLARE_CLASS(flyer_controller, ControlModeAttitude, flyer_controller::ControlModeAttitude, nodelet::Nodelet)

} // namespace
