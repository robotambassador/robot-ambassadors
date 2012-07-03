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
#include "flyer_controller/attitude_mode.h"
namespace flyer_controller
{

AttitudeController::AttitudeController() :
  yaw_cmd(0), max_pitch_cmd(10), max_roll_cmd(10), external_command_frame(false), external_frame_heading(0)
{
}

void AttitudeController::outputControl(const nav_msgs::OdometryConstPtr& latest_state,
                                       const joystick_command& latest_cmd, const joy::JoyConstPtr& latest_joy,
                                       control_mode_output& control_out)

{
  ros::Time now_time = ros::Time::now();
  double dt = (now_time - last_time).toSec();

  double roll_cmd, pitch_cmd, alt_cmd;
  double roll_cmd_rot, pitch_cmd_rot;
  double relative_angle, cos_ang, sin_ang;

  roll_cmd = latest_cmd.roll * max_roll_cmd;
  pitch_cmd = latest_cmd.pitch * max_pitch_cmd;

  double yaw_rate_cmd = std::numeric_limits<double>::quiet_NaN();
  if (latest_joy->buttons[0] == 1)
  {
    if (not direct_yaw_rate_control)
    {
      yaw_cmd = yaw_cmd + latest_cmd.yaw * max_yaw_rate_cmd * dt;
      yaw_rate_cmd = std::numeric_limits<double>::quiet_NaN();
    }
    else
    {
      yaw_cmd = std::numeric_limits<double>::quiet_NaN();
      yaw_rate_cmd = latest_cmd.yaw * max_yaw_rate_cmd;
    }
  }
  else
  {
    if (not direct_yaw_rate_control)
    {
      yaw_rate_cmd = std::numeric_limits<double>::quiet_NaN();
    }
    else
    {
      yaw_rate_cmd = 0;
    }
  }
  alt_cmd = min_alt_cmd + (max_alt_cmd - min_alt_cmd) * latest_cmd.alt;
  if (external_command_frame)
  {
    double current_yaw, current_pitch, current_roll;
    odom_msg_to_ypr(latest_state, current_yaw, current_pitch, current_roll);
    relative_angle = current_yaw - angles::from_degrees(external_frame_heading);
    cos_ang = cos(relative_angle);
    sin_ang = sin(relative_angle);
    roll_cmd_rot = cos_ang * roll_cmd + sin_ang * pitch_cmd;
    pitch_cmd_rot = -sin_ang * roll_cmd + cos_ang * pitch_cmd;
  }
  else
  {
    roll_cmd_rot = roll_cmd;
    pitch_cmd_rot = pitch_cmd;
  }
  control_out.roll_cmd = roll_cmd_rot;
  control_out.pitch_cmd = pitch_cmd_rot;
  control_out.alt_cmd = alt_cmd;
  control_out.yaw_cmd = yaw_cmd;
  control_out.yaw_rate_cmd = yaw_rate_cmd;
  control_out.motors_on = (latest_joy->buttons[0] == 1);

  last_time = now_time;
}

void AttitudeController::useCurrentYaw(const nav_msgs::OdometryConstPtr& latest_state)
{
  double current_yaw, current_pitch, current_roll;
  if (external_command_frame) // TODO: does this make sense??
  {
    odom_msg_to_ypr(latest_state, current_yaw, current_pitch, current_roll);
    setYawCmd(to_degrees(current_yaw));
  }
  else
  {
    setYawCmd(std::numeric_limits<double>::quiet_NaN());
  }

}
} // namespace
