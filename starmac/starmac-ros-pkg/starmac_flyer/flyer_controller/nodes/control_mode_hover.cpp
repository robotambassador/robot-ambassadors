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
#include "flyer_controller/control_mode.h"
#include "flyer_controller/hover_modes.h"
// msg:
#include "flyer_controller/control_mode_hover_info.h"

namespace flyer_controller
{
class ControlModeHover : public HoverMode
{
protected:
  // Parameters
  double pitch_deadband; // joystick units, 0.1 = 10% of half-range
  double roll_deadband; // joystick units, 0.1 = 10% of half-range
  double yaw_deadband; // joystick units, 0.1 = 10% of half-range

  double max_yaw_rate_cmd; // [deg/s] yaw rate corresponding to full deflection
  double waypoint_speed; // [m/s] lateral speed at which to move the controller setpoint between autosequence points
  bool external_command_frame; // should commands be interpreted w.r.t. an external frame?
  double external_frame_heading; // [deg] heading that will correspond to a -pitch command when using external frame

  // Members
  double latest_pitch_cmd; // -1..1 (clipped, deadbanded, scaled)
  double latest_roll_cmd; // -1..1 (clipped, deadbanded, scaled)
  double latest_yaw_cmd; // -1..1 (clipped, deadbanded, scaled)
  double yaw_cmd; // degrees

public:
  ControlModeHover() :
    pitch_deadband(0.05), roll_deadband(0.05), yaw_deadband(0.1), max_yaw_rate_cmd(10), waypoint_speed(0.25),
        external_command_frame(false), external_frame_heading(0), HoverMode("hover")
  {
  }

  void onInit()
  {
    HoverMode::onInit();
    NODELET_WARN("Joystick must be properly calibrated using jscal.. have you done this?");
    // Parameters
    nh_priv.param("pitch_deadband", pitch_deadband, pitch_deadband);
    nh_priv.param("roll_deadband", roll_deadband, roll_deadband);
    nh_priv.param("yaw_deadband", yaw_deadband, yaw_deadband);
    CHECK_PARAMETER((pitch_deadband >= 0) and (pitch_deadband <= 1), "parameter value out of range");
    CHECK_PARAMETER((roll_deadband >= 0) and (roll_deadband <= 1), "parameter value out of range");
    CHECK_PARAMETER((yaw_deadband >= 0) and (yaw_deadband <= 1), "parameter value out of range");
    nh_priv.param("max_yaw_rate_cmd", max_yaw_rate_cmd, max_yaw_rate_cmd);
    CHECK_PARAMETER(max_yaw_rate_cmd >= 0, "parameter value out of range");
    nh_priv.param("waypoint_speed", waypoint_speed, waypoint_speed);
    nh_priv.param("external_command_frame", external_command_frame, external_command_frame);
    nh_priv.param("external_frame_heading", external_frame_heading, external_frame_heading);
  }

  //  void reportStatusTimerCallback(const ros::TimerEvent& e)
  //  {
  //    control_mode_status msg;
  //    //NODELET_INFO_STREAM(__PRETTY_FUNCTION__);
  //    msg.state = state;
  //    msg.info = info;
  //    ready = (got_first_joy and got_first_state and seen_max_alt and seen_min_alt and latest_alt_cmd < 0.05);
  //    //    NODELET_INFO_STREAM("got_first_joy = " << got_first_joy << " got_first_state = " << got_first_state
  //    //        << "seen_max_alt = " << seen_max_alt << " seen_min_alt = " << seen_min_alt << " latest_alt_cmd = " << latest_alt_cmd);
  //    msg.ready = ready; // TODO: check some more conditions..
  //    control_mode_status_pub.publish(msg);
  //    diag_updater.update();
  //  }

  void process_joystick(const joy::JoyConstPtr & joy_msg)
  {
    static ros::Time last_time = ros::Time::now();
    ros::Time now_time = ros::Time::now();
    double dt = (now_time - last_time).toSec();

    double roll_in = double(-joy_msg->axes[0]);
    double pitch_in = double(-joy_msg->axes[1]);
    double yaw_in = double(-joy_msg->axes[2]);
    double roll_sign = (roll_in > 0) ? 1.0 : -1.0;
    double pitch_sign = (pitch_in > 0) ? 1.0 : -1.0;
    double yaw_sign = (yaw_in > 0) ? 1.0 : -1.0;
    latest_roll_cmd = roll_sign * roll_in > roll_deadband ? (max(-1.0, min(1.0, roll_in)) - roll_sign * roll_deadband)
        / (1.0 - roll_deadband) : 0.0;
    latest_pitch_cmd = pitch_sign * pitch_in > pitch_deadband ? (max(-1.0, min(1.0, pitch_in)) - roll_sign
        * pitch_deadband) / (1.0 - pitch_deadband) : 0.0;
    latest_yaw_cmd = yaw_sign * yaw_in > yaw_deadband ? (max(-1.0, min(1.0, yaw_in)) - yaw_sign * yaw_deadband) / (1.0
        - yaw_deadband) : 0.0;

    // Now, interpret pitch and roll as translational velocity commands:
    double east_cmd_normalized, north_cmd_normalized;
    double cos_ang, sin_ang;
    if (external_command_frame)
    {
      cos_ang = cos(angles::from_degrees(external_frame_heading));
      sin_ang = sin(angles::from_degrees(external_frame_heading));
    }
    else
    {
      double current_yaw, current_pitch, current_roll;
      odom_msg_to_ypr(boost::make_shared<nav_msgs::Odometry>(latest_state), current_yaw, current_pitch, current_roll);
      cos_ang = cos(current_yaw);
      sin_ang = sin(current_yaw);
    }
    east_cmd_normalized = cos_ang * latest_roll_cmd - sin_ang * latest_pitch_cmd;
    north_cmd_normalized = -sin_ang * latest_roll_cmd - cos_ang * latest_pitch_cmd;

    north_vel_cmd = 0.0;
    east_vel_cmd = 0.0;
    if ((latest_joy.buttons[0] == 1) and (latest_joy.buttons[5] == 1))
    {
      north_vel_cmd = north_cmd_normalized * waypoint_speed;
      east_vel_cmd = east_cmd_normalized * waypoint_speed;
      north_cmd = north_cmd + north_vel_cmd * dt;
      east_cmd = east_cmd + east_vel_cmd * dt;
    }

    last_time = now_time;
  }

}; // class
PLUGINLIB_DECLARE_CLASS(flyer_controller, ControlModeHover, flyer_controller::ControlModeHover, nodelet::Nodelet)
;

} // namespace
