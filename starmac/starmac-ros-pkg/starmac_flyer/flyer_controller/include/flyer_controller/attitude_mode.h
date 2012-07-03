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
#ifndef ATTITUDE_MODE_H
#define ATTITUDE_MODE_H

#include "flyer_controller/control_mode.h"
#include "flyer_controller/joystick_modes.h"
#include "flyer_controller/control_utils.h"

using angles::to_degrees;
using angles::from_degrees;

namespace flyer_controller
{
class AttitudeController
{
private:
  ros::Time last_time;
  bool external_command_frame;
  bool direct_yaw_rate_control;
  double external_frame_heading;
  double yaw_cmd;
  double max_pitch_cmd; // [deg] commanded pitch corresponding to full deflection
  double max_roll_cmd; // [deg] commanded roll corresponding to full deflection
  double max_yaw_rate_cmd; // [deg/s] yaw rate corresponding to full deflection
  double min_alt_cmd;
  double max_alt_cmd;

public:

  AttitudeController();

  void outputControl(const nav_msgs::OdometryConstPtr& latest_state, const joystick_command& latest_cmd,
                     const joy::JoyConstPtr& latest_joy, control_mode_output& control_out);

  void configure(bool external_frame, double external_heading, bool use_direct_yaw_rate_control,
                 double max_pitch_cmd_set, double max_roll_cmd_set, double max_yaw_rate_cmd_set,
                 double min_alt_cmd_set, double max_alt_cmd_set)
  {
    external_command_frame = external_frame;
    external_frame_heading = external_heading;
    direct_yaw_rate_control = use_direct_yaw_rate_control;
    max_pitch_cmd = max_pitch_cmd_set;
    max_roll_cmd = max_roll_cmd_set;
    max_yaw_rate_cmd = max_yaw_rate_cmd_set;
    min_alt_cmd = min_alt_cmd_set;
    max_alt_cmd = max_alt_cmd_set;
  }

  void setYawCmd(double new_yaw_cmd)
  {
    yaw_cmd = new_yaw_cmd;
  }

  void useCurrentYaw(const nav_msgs::OdometryConstPtr& latest_state);
}; // class
} // namespace
#endif //
