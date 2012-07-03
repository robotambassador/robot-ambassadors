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
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include "flyer_controller/control_mode.h"
#include "flyer_controller/control_mode_hover_info.h"
#include "flyer_controller/control_utils.h"
#include "control_toolbox/pid.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

using std::string;
using std::vector;
using std::map;
using std::max;
using std::min;
using angles::to_degrees;
using angles::from_degrees;

namespace flyer_controller
{

struct hover_point
{
  string name;
  double north; // [m]
  double east; // [m]
  double north_vel; // [m/s]
  double east_vel; // [m/s]
  double yaw; // [deg]
};

class HoverMode : public ControlMode
{
protected:
  // Params
  double max_alt_cmd; // [m] commanded altitude corresponding to full + deflection
  double min_alt_cmd; // [m] commanded altitude corresponding to full - deflection
  // next two meaningless unless/until 'manual hover' implemented:
  //  bool external_command_frame; // should commands be interpreted w.r.t. an external frame?
  //  double external_frame_heading; // [deg] heading that will correspond to a -pitch command when using external frame
  double KP; // deg/m
  double KI; // deg/(m-s)
  double KD; // deg/m/s
  double Ilimit; // deg
  // Publisher
  ros::Publisher info_pub;
  ros::Publisher marker_pub;
  ros::Publisher marker_array_pub;
  // Member vars
  string mode_name_;
  control_mode_output control_out;
  double yaw_cmd; // [deg]
  control_toolbox::Pid north_pid;
  control_toolbox::Pid east_pid;
  double north_cmd;
  double east_cmd;
  double north_vel_cmd;
  double east_vel_cmd;
  double north_err;
  double east_err;
  double north_vel_err;
  double east_vel_err;
  map<string, hover_point> hover_points;
  hover_point current_hover_point;

public:

  HoverMode(string mode_name) :
        //    external_command_frame(false), external_frame_heading(0),
        max_alt_cmd(1.5), min_alt_cmd(0.0), KP(12), KI(1), KD(8), Ilimit(2), mode_name_(mode_name), yaw_cmd(0),
        north_cmd(0), east_cmd(0), north_vel_cmd(0), east_vel_cmd(0)
  {

  }

  void onInit()
  {
    NODELET_INFO("ControlModeHover onInit() called");
    ControlMode::onInit();
    // Parameters
    nh_priv.param("max_alt_cmd", max_alt_cmd, max_alt_cmd);
    nh_priv.param("min_alt_cmd", min_alt_cmd, min_alt_cmd);
    nh_priv.param("KP", KP, KP);
    nh_priv.param("KI", KI, KI);
    nh_priv.param("KD", KD, KD);
    nh_priv.param("Ilimit", Ilimit, Ilimit);

    // Publishers
    marker_pub = nh_priv.advertise<visualization_msgs::Marker> ("marker", 10, true);
    marker_array_pub = nh_priv.advertise<visualization_msgs::MarkerArray> ("marker_array", 10, true);

    // North and East PID control
    set_gains(KP, KI, KD, Ilimit);

    requestRegistration(mode_name_);
    control_out.control_mode = mode_name_;
    control_out.motors_on = false;
  }

private:
  // Parses (and acts upon if appropriate) a control_mode_command.Returns true if the command
  // was recognized and false otherwise. Note that an invalid (but still recognized) command
  // should only emit an error message but *not* also return false.
  bool parseControlModeCmd(const string cmd)
  {
    vector < string > words;
    boost::split(words, cmd, boost::is_any_of(" "));
    int nw = words.size();
    if (nw > 0)
    {
      if (words[0] == "mode")
      {
        parse_mode( words);
      }
      else if (words[0] == "define")
      // define hover_point point_name north_coord east_coord [north_vel east_vel [yaw_coord]]

      {
        if (words[1] == "hover_point")
        {
          parse_define_hover_point( words);
        }
        else
        {
          return false;
        }
      }
      else if (words[0] == "set")
      {
        if (nw > 1)
        {
          if (words[1] == "hover_point")
          {
            parse_set_hover_point( words);

          }
          else if (words[1] == "gains")
          {
            parse_set_gains( words);
          }
        }
        else
        {
          return false;
        }
      }
      else
      {
        return false;
      }
    }
    return true;
  }

protected:
  // Actual hover mode classes should implement this method if they are to respond
  // to additional commands. It is called if parseControlModeCmd returns false.
  // The default implementation returns false so that an unrecognized command
  // will result in an error even if the derived class doesn't implement this method.
  virtual bool parseControlModeCmdDerived(const string cmd)
  {
    return false;
  }

private:
  void controlModeCmdCallback(const control_mode_cmdConstPtr& msg)
  {
    NODELET_INFO_STREAM("Heard command: " << msg->cmd);
    if (!parseControlModeCmd(msg->cmd))
    {
      if (!parseControlModeCmdDerived(msg->cmd))
      {
        NODELET_ERROR("Unrecognized command");
      }
    }
  }

  void joyCallback(const joy::JoyConstPtr& msg)
  {
    NODELET_DEBUG_STREAM(__PRETTY_FUNCTION__ << " callback called");
    ControlMode::joyCallback(msg);
    if (seen_max_alt and seen_min_alt)
    {
      latest_alt_cmd = (min(1.0, max(-1.0, double(msg->axes[3]))) + 1.0) / 2.0; // clip, offset, scale to 0..1 range
    }
    process_joystick(msg);
  }

  virtual void process_joystick(const joy::JoyConstPtr & joy_msg) = 0;

  // Virtual methods from base class
  void outputControlTimerCallback(const ros::TimerEvent& e)
  {
    //control_mode_output control_out;
    bool do_calcs = false;
    bool do_publish = false;
    static ros::Time last_time = ros::Time::now();
    ros::Time now_time = ros::Time::now();
    ros::Duration dt = now_time - last_time;
    static int prev_trigger = 0;

    switch (state)
    {
      case ControlModeTypes::STANDBY:
        do_calcs = (got_first_joy and got_first_state);
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
      double current_yaw, current_pitch, current_roll; // radians
      double current_vx, current_vy, current_vz;
      odom_msg_to_ypr(boost::make_shared<nav_msgs::Odometry>(latest_state), current_yaw, current_pitch, current_roll);
      odom_msg_to_lin_vel(boost::make_shared<nav_msgs::Odometry>(latest_state), current_vx, current_vy, current_vz);
      get_current_lateral_position_errors(north_err, east_err);
      north_vel_err = current_vx - north_vel_cmd;
      east_vel_err = current_vy - east_vel_cmd;
      if (latest_joy.buttons[0] == 1 and prev_trigger == 0)
      {
        north_pid.reset();
        east_pid.reset();
      }
      north_pid.updatePid(north_err, north_vel_err, dt);
      east_pid.updatePid(east_err, east_vel_err, dt);
      double north_cmd = north_pid.getCurrentCmd();
      double east_cmd = east_pid.getCurrentCmd();
      // Rotate into body frame:
      double cos_yaw = cos(current_yaw);
      double sin_yaw = sin(current_yaw);
      control_out.roll_cmd = east_cmd * cos_yaw - north_cmd * sin_yaw;
      control_out.pitch_cmd = -east_cmd * sin_yaw - north_cmd * cos_yaw;
      prev_trigger = latest_joy.buttons[0];
    }
    control_out.yaw_cmd = yaw_cmd;
    control_out.yaw_rate_cmd = std::numeric_limits<double>::quiet_NaN();
    control_out.alt_cmd = min_alt_cmd + (max_alt_cmd - min_alt_cmd) * latest_alt_cmd;
    control_out.motors_on = (got_first_joy and (latest_joy.buttons[0] == 1));
    if (do_publish)
    {
      control_out.control_mode = "hover";
      output_pub.publish(control_out);
    }

    last_time = now_time;
  }

protected:
  void get_current_lateral_position_errors(double &north_err, double &east_err)
  {
    double current_x, current_y, current_z;
    odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
    north_err = current_x - north_cmd;
    east_err = current_y - east_cmd;
  }

  void get_current_lateral_position(double &current_x, double &current_y)
  {
    double current_z;
    odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
  }

  void get_current_error_to_point(const hover_point& point, double &north_err, double &east_err, double &yaw_err)
  {
    double current_x, current_y, current_z;
    odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
    north_err = current_x - point.north;
    east_err = current_y - point.east;
    double current_yaw, current_pitch, current_roll;
    odom_msg_to_ypr(boost::make_shared<nav_msgs::Odometry>(latest_state), current_yaw, current_pitch, current_roll);
    //    NODELET_INFO_STREAM("yaw=" << to_degrees(current_yaw) << " pitch="
    //                    << to_degrees(current_pitch) << " roll=" << to_degrees(current_roll));
    yaw_err = angles::shortest_angular_distance(current_yaw, angles::from_degrees(point.yaw));
  }

private:
  void reportStatusTimerCallback(const ros::TimerEvent& e)
  {
    control_mode_status msg;
    //NODELET_INFO_STREAM(__PRETTY_FUNCTION__);
    msg.state = state;
    msg.info = info;
    ready = (got_first_joy and got_first_state and seen_max_alt and seen_min_alt and latest_alt_cmd < 0.05);
    //    NODELET_INFO_STREAM("got_first_joy = " << got_first_joy << " got_first_state = " << got_first_state);
    msg.ready = ready; // TODO: check some more conditions..
    control_mode_status_pub.publish(msg);

    if ((state == ControlModeTypes::STANDBY) or (state == ControlModeTypes::ACTIVE))
    {
      control_mode_hover_info info_msg;
      info_msg.hover_point = current_hover_point.name;
      info_msg.north_cmd = north_cmd;
      info_msg.east_cmd = east_cmd;
      info_msg.yaw_cmd = yaw_cmd;
      info_msg.north_err = north_err;
      info_msg.east_err = east_err;
      info_msg.north_vel_err = north_vel_err;
      info_msg.east_vel_err = east_vel_err;
      info_msg.yaw_cmd = yaw_cmd;
      // TODO: add errors..
      info_pub.publish(info_msg);
    }
    diag_updater.update();
  }

  // Command parsers
  void parse_mode(const vector<string> &words)
  {
    int nw = words.size();
    if (nw == 2)
    {
      string newmode = words[1];
      if (newmode == "idle")
      {
        state = ControlModeTypes::IDLE;
        info = "";
      }
      else if (newmode == "standby")
      {
        info = "";
        info_pub = nh_priv.advertise<control_mode_hover_info> ("info", 10);
        state = ControlModeTypes::STANDBY;
      }
      else if (newmode == "active")
      {
        if (state == ControlModeTypes::STANDBY)
        {
          state = ControlModeTypes::ACTIVE;
          double current_yaw, current_pitch, current_roll;
          double current_x, current_y, current_z;
          odom_msg_to_ypr(boost::make_shared<nav_msgs::Odometry>(latest_state), current_yaw, current_pitch,
                          current_roll);
          odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
          yaw_cmd = to_degrees(current_yaw);
          north_cmd = current_x;
          east_cmd = current_y;
          info = "";
        }
      }
      else
      {
        NODELET_ERROR("Unknown mode");
      }
    }
    else
    {
      NODELET_ERROR("Invalid command");
    }
  }

  void parse_define_hover_point(const vector<string> &words)
  {
    int nw = words.size();
    if (not ((nw == 5) or (nw == 7) or (nw == 8)))
    {
      NODELET_ERROR("Invalid command");
    }
    else
    {
      string point_name = words[2];
      if (hover_point_exists(point_name))
      {
        NODELET_WARN_STREAM("hover_point '" << point_name << "' already exists and will be redefined");
      }
      hover_points[point_name].name = point_name;
      hover_points[point_name].north = atof(words[3].c_str());
      hover_points[point_name].east = atof(words[4].c_str());
      if ((nw == 7) or (nw == 8)) // north_vel, east_vel and perhaps yaw_coord specified:

      {
        hover_points[point_name].north_vel = atof(words[5].c_str());
        hover_points[point_name].east_vel = atof(words[6].c_str());
        if (nw == 7)
        {
          hover_points[point_name].yaw = NAN;
          NODELET_INFO_STREAM("New hover_point defined: name='" << point_name << "' north="
              << hover_points[point_name].north << " east=" << hover_points[point_name].east << " north_vel="
              << hover_points[point_name].north_vel << " east_vel=" << hover_points[point_name].east_vel
              << ". Yaw unspecified.");
        }
        else if (nw == 8)
        {
          hover_points[point_name].yaw = atof(words[7].c_str());
          NODELET_INFO_STREAM("New hover_point defined: name='" << point_name << "' north="
              << hover_points[point_name].north << " east=" << hover_points[point_name].east << " north_vel="
              << hover_points[point_name].north_vel << " east_vel=" << hover_points[point_name].east_vel << " yaw="
              << hover_points[point_name].yaw);
        }
      }
      else
      {
        hover_points[point_name].north_vel = 0;
        hover_points[point_name].east_vel = 0;
        hover_points[point_name].yaw = NAN;
        NODELET_INFO_STREAM("New hover_point defined: name='" << point_name << "' north="
            << hover_points[point_name].north << " east=" << hover_points[point_name].east << " north_vel="
            << hover_points[point_name].north_vel << " east_vel=" << hover_points[point_name].east_vel
            << ". Yaw unspecified.");
      }
    }
  }

protected:
  bool hover_point_exists(const string point_name)
  {
    map<string, hover_point>::iterator it;
    it = hover_points.find(point_name);
    return (not (it == hover_points.end()));
  }

  void set_gains(double KP, double KI, double KD, double Ilimit)
  {
    north_pid.initPid(KP, KI, KD, Ilimit, -Ilimit);
    east_pid.initPid(KP, KI, KD, Ilimit, -Ilimit);
  }

  // syntax:
  // set gains KP KI KD Ilimit
  void parse_set_gains(const vector<string> &words)
  {
    int nw = words.size();
    if (nw == 6)
    {
      double KP_new = atof(words[2].c_str());
      double KI_new = atof(words[3].c_str());
      double KD_new = atof(words[4].c_str());
      double Ilimit_new = atof(words[5].c_str());
      // TODO: do some checks on what was entered
      KP = KP_new;
      KI = KI_new;
      KD = KD_new;
      Ilimit = Ilimit_new;
      set_gains(KP, KI, KD, Ilimit);
      NODELET_INFO("Set new gains: KP=%f, KI=%f, KD=%f, Ilimit=%f", KP, KI, KD, Ilimit);
    }
    else
    {
      NODELET_ERROR("Invalid command");
    }
  }

  void parse_set_hover_point(const vector<string> &words)
  {
    int nw = words.size();
    if (nw == 3)
    {
      string point_name = words[2];
      if (hover_point_exists(point_name))
      {
        set_hover_point(hover_points[point_name]);
      }
      else
      {
        NODELET_ERROR("Invalid hover_point name");
      }
    }
    else
    {
      NODELET_ERROR("Invalid command");
    }
  }

  void set_hover_point(const hover_point& point, bool reset = false)
  {
    north_cmd = point.north;
    east_cmd = point.east;
    north_vel_cmd = point.north_vel;
    east_vel_cmd = point.east_vel;
    if (reset)
    {
      north_pid.reset();
      east_pid.reset();
    }
    current_hover_point = point;
    //    NODELET_INFO_STREAM("Setting hover setpoint to hover_point '" << point.name << "': north="
    //        << north_cmd << " east=" << east_cmd
    //        << " north_vel_cmd=" << north_vel_cmd << " east_vel_cmd=" << east_vel_cmd);
    if (not isnan(point.yaw))
    {
      yaw_cmd = point.yaw;
      //      NODELET_INFO_STREAM("Setting yaw setpoint to " << yaw_cmd);
    }
    else
    {
      //      NODELET_INFO("Hover point does not define a yaw value, yaw command unmodified.");
    }
    send_viz_marker(north_cmd, east_cmd, yaw_cmd);
  }

  void send_viz_marker(double north, double east, double yaw)
  {
    if (marker_pub.getNumSubscribers())
    {
      visualization_msgs::Marker waypoint, arrow;
      arrow.header.frame_id = waypoint.header.frame_id = "/ned";
      arrow.header.stamp = waypoint.header.stamp = ros::Time::now();
      waypoint.ns = "waypoint";
      arrow.ns = "arrow";
      arrow.action = waypoint.action = visualization_msgs::Marker::ADD;
      waypoint.pose.position.x = north;
      waypoint.pose.position.y = east;
      waypoint.pose.position.z = 0;
      waypoint.pose.orientation.w = 1.0;
      waypoint.id = 0;
      arrow.id = 0;
      waypoint.type = visualization_msgs::Marker::SPHERE;
      arrow.type = visualization_msgs::Marker::ARROW;
      waypoint.scale.x = 0.2;
      waypoint.scale.y = 0.2;
      waypoint.scale.z = 0.2;
      arrow.scale.x = 0.05;
      arrow.scale.y = 0.10;
      waypoint.color.r = 1.0;
      waypoint.color.g = 1.0;
      waypoint.color.b = 1.0;
      waypoint.color.a = 0.8;
      arrow.color.r = 1.0;
      arrow.color.g = 0.0;
      arrow.color.b = 0.0;
      arrow.color.a = 0.8;
      geometry_msgs::Point p;
      p.z = 0;
      arrow.points.push_back(p);
      arrow.points.push_back(p);
      arrow.points[0].x = north;
      arrow.points[0].y = east;
      arrow.points[1].x = north + 0.25 * cos(from_degrees(yaw));
      arrow.points[1].y = east + +0.25 * sin(from_degrees(yaw));
      waypoint.lifetime = ros::Duration();
      marker_pub.publish(waypoint);
      marker_pub.publish(arrow);
    }
  }

};

}
