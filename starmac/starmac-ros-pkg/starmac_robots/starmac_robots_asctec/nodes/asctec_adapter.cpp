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
#include <algorithm>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <dynamic_reconfigure/server.h>
#include <starmac_robots_asctec/AscTecAdapterConfig.h>
#include <string>
#include <angles/angles.h>
#include "asctec_msgs/CtrlInput.h"
#include "asctec_msgs/LLStatus.h"
#include "std_msgs/Bool.h"
#include "flyer_controller/control_mode_output.h"
#include "control_toolbox/pid.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;

// From AscTec documentation:
static short int CTRL_PITCH = 1;
static short int CTRL_ROLL = 2;
static short int CTRL_YAW = 4;
static short int CTRL_THRUST = 8;
static short int CTRL_NONE = 0;
static short int CTRL_ALL = CTRL_PITCH | CTRL_ROLL | CTRL_YAW | CTRL_THRUST; // 15
// Per email from AscTec, "25 millidegree" per count:
// """The scaling of the roll/pitch/yaw axes is linear. Multiply the values
// with 25 millidegree (+-2047 * 25 millidegree = +-51.175 degree)."""
static double ROLL_SCALE = 40; // counts/deg
static double PITCH_SCALE = 40; // counts/deg
// Per email from AscTec,
// """The standard parameter for K_stick_yaw is 120, resulting in a maximum rate of
// 254.760 degrees per second. I.e. a 360° turn takes about 1.5 seconds."""
static double YAW_SCALE = 2047 / 254.760; // counts/deg/s

namespace starmac_robots_asctec
{

class AscTecAdapter
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;
  double min_freq;
  double max_freq;
  diagnostic_updater::FrequencyStatus freq_status;
  // Dynamic Reconfigure Server
  dynamic_reconfigure::Server<starmac_robots_asctec::AscTecAdapterConfig> dyn_reconfig_srv;
  // Parameters
  string output_topic; // Topic on which to publish output
  string estop_topic; // Topic on which to publish emergency stop command
  ros::Duration deadman_timeout; // timeout in seconds
  double alt_KP; // thrust counts per meter
  double alt_KI; // thrust counts per meter-second
  double alt_KD; // thrust counts per m/s
  double alt_KDD; // thrust counts per m/s/s
  double alt_Ilimit; // thrust counts
  double yaw_KP; // deg/s per deg
  double yaw_KI; // deg/s per deg-second
  double yaw_KD; // deg/s per deg/s
  double yaw_Ilimit; // deg/s
  double yaw_rate_limit; // deg/s
  double roll_slew_rate_limit; // deg/s/s
  double pitch_slew_rate_limit; // deg/s/s
  double yaw_slew_rate_limit; // deg/s/s
  double alt_slew_rate_limit; // m/s/s
  int nominal_thrust; // [0..4095] TODO: change to mass?
  double thrust_mult; // 1.0 means full power
  int max_thrust; // [0..4095]
  double accel_bias; // m/s/s - for a perfect sensor this would be 9.81
  unsigned int land_now_thrust_decrease_rate; // [counts/s] - how fast to decrease commanded thrust
  double land_now_min_thrust_ratio; // reduce thrust to this level (as fraction of nominal_thrust) then keep it constant
  double pitch_trim; // deg - will be added to commanded pitch (AscTec sign convention)
  double roll_trim; // deg - will be added to commanded roll (AscTec sign convention)
  double yaw_offset; // deg
  // Publishers
  ros::Publisher output;
  ros::Publisher estop_pub;
  // Subscribers
  ros::Subscriber motor_enable_sub;
  ros::Subscriber control_input_sub;
  ros::Subscriber state_sub;
  ros::Subscriber ll_status_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber land_now_sub;
  // Timers
  ros::Timer deadman_timer;
  // Members
  bool last_motor_enable;
  bool deadman; // latches to true if motor enable not seen for more than deadman_timeout
  ros::Time last_control_input_time;
  ros::Time last_motor_enable_time;
  ros::Time last_ll_status_time;
  ros::Time last_state_time;
  ros::Time last_imu_time;
  control_toolbox::Pid alt_pid;
  control_toolbox::Pid yaw_pid;
  nav_msgs::Odometry latest_state;
  asctec_msgs::LLStatus latest_ll_status;
  asctec_msgs::CtrlInput zero_thrust_full_left_yaw;
  asctec_msgs::CtrlInput null_input;
  flyer_controller::control_mode_output last_control_input;
  ros::Duration max_motor_enable_age;
  ros::Duration max_ll_status_age;
  ros::Duration max_control_input_age;
  ros::Duration max_state_age;
  double alt_p_term;
  double alt_i_term;
  double alt_d_term;
  double alt_dd_term;
  double last_z_accel;
  bool land_now;
  bool estop_sent;
  bool got_first_state;

public:
  AscTecAdapter() :
        nh_priv("~"),
        diag_updater(),
        min_freq(0.1),
        max_freq(1000),
        freq_status(diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq)),
        // Params:
        output_topic("autopilot/CTRL_INPUT"), estop_topic("autopilot/ESTOP"), deadman_timeout(0.5), alt_KP(200.0),
        alt_KI(15.0), alt_KD(300.0), alt_KDD(0.0), alt_Ilimit(5.0), yaw_KP(1.0), yaw_KI(1.0), yaw_KD(0.0),
        yaw_Ilimit(5.0), yaw_rate_limit(4.0), roll_slew_rate_limit(10), pitch_slew_rate_limit(10),
        yaw_slew_rate_limit(5), alt_slew_rate_limit(0.1),
        nominal_thrust(1950),
        thrust_mult(1.0),
        max_thrust(2100),
        accel_bias(9.81),
        // Members:
        last_motor_enable(false), deadman(false), last_control_input_time(0), last_motor_enable_time(0),
        last_ll_status_time(0), last_state_time(0), max_motor_enable_age(ros::Duration(0)),
        max_ll_status_age(ros::Duration(0)), max_control_input_age(ros::Duration(0)), max_state_age(ros::Duration(0)),
        land_now_thrust_decrease_rate(200), land_now_min_thrust_ratio(0.5), pitch_trim(0), roll_trim(0), yaw_offset(0),
        land_now(false), estop_sent(false), got_first_state(false)
  {
    initDiagnostics();
    initDynamicReconfigure();
    initParameters();
    initPublishers();
    initSubscribers();
    initTimers();
    // Misc
    zero_thrust_full_left_yaw = assemble_command(0, 0, -2047 / YAW_SCALE, 0, true, true, true, true);
    null_input = assemble_command(0, 0, 0, 0, false, false, false, false);
  }

private:

  void initDiagnostics()
  {
    // Diagnostics
    diag_updater.add("AscTecAdapter Status", this, &AscTecAdapter::diagnostics);
    diag_updater.add(freq_status);
    diag_updater.setHardwareID("none");
    diag_updater.force_update();
  }

  void initDynamicReconfigure()
  {
    // Dynamic Reconfigure Server
    dynamic_reconfigure::Server<starmac_robots_asctec::AscTecAdapterConfig>::CallbackType f =
        boost::bind(&AscTecAdapter::cfgCallback, this, _1, _2);
    dyn_reconfig_srv.setCallback(f);
  }

  void initParameters()
  {
    // Parameters
    nh_priv.param("output_topic", output_topic, output_topic);
    nh_priv.param("estop_topic", estop_topic, estop_topic);
    double temp = deadman_timeout.toSec();
    nh_priv.param("deadman_timeout", temp, temp);
    deadman_timeout.fromSec(temp);
    nh_priv.param("alt_KP", alt_KP, alt_KP);
    nh_priv.param("alt_KI", alt_KI, alt_KI);
    nh_priv.param("alt_KD", alt_KD, alt_KD);
    nh_priv.param("alt_KDD", alt_KDD, alt_KDD);
    nh_priv.param("alt_Ilimit", alt_Ilimit, alt_Ilimit);
    nh_priv.param("yaw_KP", yaw_KP, yaw_KP);
    nh_priv.param("yaw_KI", yaw_KI, yaw_KI);
    nh_priv.param("yaw_KD", yaw_KD, yaw_KD);
    nh_priv.param("yaw_Ilimit", yaw_Ilimit, yaw_Ilimit);
    nh_priv.param("yaw_rate_limit", yaw_rate_limit, yaw_rate_limit);
    nh_priv.param("nominal_thrust", nominal_thrust, nominal_thrust);
    nh_priv.param("thrust_mult", thrust_mult, thrust_mult);
    nh_priv.param("max_thrust", max_thrust, max_thrust);
    nh_priv.param("accel_bias", accel_bias, accel_bias);
    int temp_uint; // ROS parameters can't be unsigned
    nh_priv.param("land_now_thrust_decrease_rate", temp_uint, (int)land_now_thrust_decrease_rate);
    land_now_thrust_decrease_rate = (unsigned int)temp_uint;
    nh_priv.param("land_now_min_thrust_ratio", land_now_min_thrust_ratio, land_now_min_thrust_ratio);
    nh_priv.param("pitch_trim", pitch_trim, pitch_trim);
    nh_priv.param("roll_trim", roll_trim, roll_trim);
    nh_priv.param("yaw_offset", yaw_offset, yaw_offset);
    max_thrust = min(4095, max_thrust);
    // Altitude and Yaw PID control
    setAltControllerParams(alt_KP, alt_KI, alt_KD, alt_Ilimit);
    setYawControllerParams(yaw_KP, yaw_KI, yaw_KD, yaw_Ilimit);
  }

  void initPublishers()
  {
    // Publishers
    output = nh.advertise<asctec_msgs::CtrlInput> (output_topic, 1);
    estop_pub = nh.advertise<std_msgs::Bool> (estop_topic, 1);
  }

  void initTimers()
  {
    // Timers
    double deadman_timer_period = deadman_timeout.toSec() / 2;
    min_freq = 0.95 / deadman_timer_period;
    max_freq = 1.05 / deadman_timer_period;
    deadman_timer = nh.createTimer(ros::Duration(deadman_timer_period), &AscTecAdapter::deadmanCallback, this);
  }

  void initSubscribers()
  {
    // Subscribers
    motor_enable_sub = nh.subscribe("teleop_flyer/motor_enable", 1, &AscTecAdapter::motorEnableCallback, this,
                                    ros::TransportHints().tcpNoDelay());
    control_input_sub = nh.subscribe("controller_mux/output", 1, &AscTecAdapter::controlInputCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    state_sub = nh.subscribe("odom", 1, &AscTecAdapter::stateCallback, this, ros::TransportHints().tcpNoDelay());
    ll_status_sub = nh.subscribe("autopilot/LL_STATUS", 1, &AscTecAdapter::llStatusCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    imu_sub = nh.subscribe("asctec_proc/imu", 1, &AscTecAdapter::imuCallback, this, ros::TransportHints().tcpNoDelay());
    land_now_sub = nh_priv.subscribe("land_now", 10, &AscTecAdapter::landNowCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  }

  void cfgCallback(starmac_robots_asctec::AscTecAdapterConfig &config, uint32_t level)
  {
    ROS_INFO("Reconfigure request");
    if (isSafeToReconfigure())
    {
      setYawControllerParams(config.yaw_KP, config.yaw_KI, config.yaw_KD, config.yaw_Ilimit);
      roll_trim = config.roll_trim;
      pitch_trim = config.pitch_trim;
    }
    else
    {
      ROS_ERROR("Cannot reconfigure; isSafeToReconfigure() returned false");
    }
  }

  void setAltControllerParams(double KP, double KI, double KD, double Ilimit)
  {
    alt_pid.initPid(KP, KI, KD, Ilimit, -Ilimit);
    ROS_INFO("Altitude controller gains set: KP=%f, KI=%f, KD=%f, Ilimit=%f", KP, KI, KD, Ilimit);
  }

  void setYawControllerParams(double KP, double KI, double KD, double Ilimit)
  {
    yaw_pid.initPid(KP, KI, KD, Ilimit, -Ilimit);
    ROS_INFO("Yaw controller gains set: KP=%f, KI=%f, KD=%f, Ilimit=%f", KP, KI, KD, Ilimit);
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (deadman)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Deadman Tripped");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
    stat.add("Motor Enable", last_motor_enable);
    stat.add("Deadman Tripped", deadman);
    stat.add("max ll_status age", max_ll_status_age);
    stat.add("max control inputs age", max_control_input_age);
    stat.add("max motor_enable age", max_motor_enable_age);
    stat.add("max state age", max_state_age);
    stat.add("alt P term", alt_p_term);
    stat.add("alt I term", alt_i_term);
    stat.add("alt D term", alt_d_term);
    stat.add("alt DD term", alt_dd_term);
  }

  void motorEnableCallback(std_msgs::Bool msg)
  {
    static bool first = true;
    last_motor_enable = msg.data;
    ros::Time now = ros::Time::now();
    if (!first)
    {
      max_motor_enable_age = max(now - last_motor_enable_time, max_motor_enable_age);
    }
    else
    {
      first = false;
    }
    last_motor_enable_time = now;
  }

  void stateCallback(nav_msgs::Odometry msg)
  {
    static bool first = true;
    latest_state = msg;
    ROS_DEBUG_STREAM("Got state with z = " << latest_state.pose.pose.position.z);
    ros::Time now = ros::Time::now();
    if (!first)
    {
      max_state_age = max(now - last_state_time, max_state_age);
    }
    else
    {
      first = false;
      got_first_state = true;
    }
    last_state_time = now;
  }

  void imuCallback(sensor_msgs::Imu msg)
  {
    //    static double avg_accel = 0;
    //    static int n_meas = 0;
    ros::Time now = ros::Time::now();
    last_imu_time = now;
    btMatrix3x3 R_inertial_imu(tf::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z,
                                              msg.orientation.w));

    btVector3 acc_imu(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    btVector3 acc_inertial = R_inertial_imu * acc_imu;
    //    ROS_INFO("acc_inertial = %g, %g, %g", acc_inertial[0], acc_inertial[1], acc_inertial[2]);
    last_z_accel = acc_inertial[2] + accel_bias; // -ve values = ascending
    //    ROS_INFO("last_z_accel = %g", last_z_accel);
    //    ROS_INFO("norm of accel = %g (raw), %g (rotated), %g (avg z)", acc_imu.length(), acc_inertial.length(), avg_accel);
    //    n_meas++;
    //    avg_accel = (avg_accel * (n_meas - 1) + acc_inertial[2]) / n_meas;
  }

  void landNowCallback(const std_msgs::BoolConstPtr& msg)
  {
    land_now = msg->data;
  }

  void llStatusCallback(asctec_msgs::LLStatus msg)
  {
    static bool first = true;
    latest_ll_status = msg;
    ros::Time now = ros::Time::now();
    if (!first)
    {
      max_ll_status_age = max(now - last_ll_status_time, max_ll_status_age);
    }
    else
    {
      first = false;
    }
    last_ll_status_time = now;
  }

  void limitSlewRate(double x, double x_prev, double dt, double rate_limit, double& x_limited)
  {
    double limit = rate_limit * dt;
    x_limited = max(x_prev - limit, min(x_prev + limit, x));
  }

  void controlInputCallback(flyer_controller::control_mode_output msg)
  {
    static bool first = true;
    ros::Time now = ros::Time::now();
    ros::Duration dt = now - last_control_input_time;
    static bool prev_motor_enable = false;
    if (!first)
    {
      max_control_input_age = max(dt, max_control_input_age);
    }
    else
    {
      first = false;
    }

    double roll_cmd_limited, pitch_cmd_limited, yaw_cmd_limited, alt_cmd_limited;
    double dt_sec = dt.toSec();
    // TODO: Think about whether when limiting occurs, it should occur on all axes proportionally
    // or at least, if roll is being limited by some factor x, then pitch should also, otherwise the overall
    // direction of the commanded thrust vector will be different
    limitSlewRate(msg.roll_cmd, last_control_input.roll_cmd, dt_sec, roll_slew_rate_limit, roll_cmd_limited);
    //    if (roll_cmd_limited != msg.roll_cmd) {
    //      ROS_INFO_STREAM("Roll command was slew rate limited. Orig cmd: " << msg.roll_cmd << "; New cmd: " << roll_cmd_limited);
    //    }
    limitSlewRate(msg.pitch_cmd, last_control_input.pitch_cmd, dt_sec, pitch_slew_rate_limit, pitch_cmd_limited);
    if (not isnan(msg.yaw_cmd))
    {
      limitSlewRate(msg.yaw_cmd, last_control_input.yaw_cmd, dt_sec, yaw_slew_rate_limit, yaw_cmd_limited);
    }
    limitSlewRate(msg.alt_cmd, last_control_input.alt_cmd, dt_sec, alt_slew_rate_limit, alt_cmd_limited);
    if (last_motor_enable and not prev_motor_enable)
    {
      double pe, de, ie;
      yaw_pid.getCurrentPIDErrors(&pe, &de, &ie);
      ROS_INFO_STREAM("Yaw integrator was at:" << ie);
      alt_pid.reset();
      yaw_pid.reset();
      ROS_INFO("Reset yaw & alt integrators");
    }
    prev_motor_enable = last_motor_enable;
    int thrust_cmd = 0;
    if (got_first_state)
    {
      ROS_DEBUG("Running computeThrust, alt_cmd_limited = %f, dt = %f", alt_cmd_limited, dt.toSec());
      thrust_cmd = computeThrust(alt_cmd_limited, dt);
    } else {
      ROS_DEBUG("got_first_state was false..");
    }
    double yaw_rate_cmd = 0; // [deg/s]
    if (not land_now)
    {
      // Don't want to try and control yaw during rapid landing since once on the ground
      // (which we can't assume we have a way of sensing, since we might be landing precisely
      //  *because* we've lost our state estimate) the yaw controller will try hard to
      // take out yaw error resulting in one pair of motors spinning up.
      if (not isnan(msg.yaw_rate_cmd))
      {
        if (not isnan(msg.yaw_cmd))
        {
          ROS_WARN("Only one of yaw_rate_cmd and yaw_cmd should be set to a real value");
        }
        yaw_rate_cmd = msg.yaw_rate_cmd;
      }
      else
      {
        if (not isnan(msg.yaw_rate_cmd))
        {
          ROS_WARN("Only one of yaw_rate_cmd and yaw_cmd should be set to a real value");
        }
        if (got_first_state)
        {
          yaw_rate_cmd = computeYawRate(yaw_cmd_limited, dt);
        }
      }
    }
    else
    {
      // Also zero out pitch and roll during rapid landing
      roll_cmd_limited = roll_trim;
      pitch_cmd_limited = pitch_trim;
    }
    asctec_msgs::CtrlInput cin;
    double roll_cmd_rotated, pitch_cmd_rotated; // deg
    rotateRollAndPitch(roll_cmd_limited, pitch_cmd_limited, roll_cmd_rotated, pitch_cmd_rotated);
    roll_cmd_rotated += roll_trim;
    pitch_cmd_rotated += pitch_trim;
    if (!deadman && dt.toSec() < 1.0)
    {
      bool motors = msg.motors_on and last_motor_enable;
      if (!motors)
      {
        thrust_cmd = 0;
      }
      cin = assemble_command(roll_cmd_rotated, pitch_cmd_rotated, yaw_rate_cmd, thrust_cmd, true, true, true, true);
    }
    if (deadman)
    {
      if (not estop_sent)
      {
        std_msgs::Bool estop_msg;
        estop_msg.data = true;
        estop_pub.publish(estop_msg);
        estop_sent = true;
      }
      if (latest_ll_status.flying == 1)
      {
        cin = zero_thrust_full_left_yaw; // send motor shutoff
      }
      else
      {
        //cin = null_input;
        // Send roll, pitch, yaw values but don't enable those bits:
        cin = assemble_command(roll_cmd_limited, pitch_cmd_limited, yaw_rate_cmd, thrust_cmd, false, false, false,
                               false);
      }
    }
    cin.header.stamp = now;
    output.publish(cin);
    last_control_input_time = now;
    last_control_input = msg;
  }

  void rotateRollAndPitch(const double roll_cmd, const double pitch_cmd, double& roll_cmd_rotated,
                          double& pitch_cmd_rotated)
  {
    double cy = cos(angles::from_degrees(yaw_offset));
    double sy = sin(angles::from_degrees(yaw_offset));
    roll_cmd_rotated = cy * roll_cmd - sy * pitch_cmd;
    pitch_cmd_rotated = sy * roll_cmd + cy * pitch_cmd;
  }

  int computeThrust(double alt_cmd, ros::Duration& dt)
  {
    // TODO: probably shouldn't have statics here, they should be member vars
    // (though as long as there's only one instance of AscTecAdapter in the program
    //  it's really just academic)
    static int last_thrust = 0;
    static bool landing_now = false;
    static int land_now_start_thrust = 0;
    static ros::Time land_now_start_time;
    if (land_now)
    {
      if (!landing_now)
      {
        ROS_INFO("Received land_now instruction. Decreasing thrust at %d counts/s to %d counts",
                 land_now_thrust_decrease_rate, int(land_now_min_thrust_ratio * double(nominal_thrust)));
        land_now_start_thrust = last_thrust;
        land_now_start_time = ros::Time::now();
        landing_now = true;
      }
      last_thrust = int(max(land_now_min_thrust_ratio * double(nominal_thrust), land_now_start_thrust
          - (ros::Time::now() - land_now_start_time).toSec() * land_now_thrust_decrease_rate));
    }
    else
    {
      if (landing_now)
      {
        // We were landing but not anymore, so reset things:
        landing_now = false;
      }
      double alt_err = (-latest_state.pose.pose.position.z) - alt_cmd;
      double alt_vel_err = -latest_state.twist.twist.linear.z;
      alt_pid.updatePid(alt_err, alt_vel_err, dt);
      double pe, ie, de;
      alt_pid.getCurrentPIDErrors(&pe, &ie, &de);
      alt_p_term = alt_KP * pe;
      alt_i_term = alt_KI * ie;
      alt_d_term = alt_KD * de;
      alt_dd_term = alt_KDD * last_z_accel;
      last_thrust = int(nominal_thrust + alt_pid.getCurrentCmd() + alt_dd_term);
    }
    return last_thrust;
  }

  double computeYawRate(double yaw_cmd, ros::Duration& dt)
  {
    double yaw, pitch, roll;
    getLatestYPR(yaw, pitch, roll);
    //double yaw_err = angles::to_degrees(yaw) - yaw_cmd;
    double yaw_err = angles::to_degrees(angles::shortest_angular_distance(angles::from_degrees(yaw_cmd), yaw
        + angles::from_degrees(yaw_offset)));
    //ROS_INFO_STREAM("Yaw error = " << yaw_err << " [deg]");
    yaw_pid.updatePid(yaw_err, dt);
    double yaw_rate_cmd = min(yaw_rate_limit, max(-yaw_rate_limit, yaw_pid.getCurrentCmd()));
    //ROS_INFO_STREAM("Yaw rate command = " << yaw_rate_cmd << " [deg/s]");
    return yaw_rate_cmd;
  }

  void getLatestYPR(double& yaw, double& pitch, double& roll)
  {
    tf::Quaternion temp_q;
//    ROS_INFO("About to do tf::quaternionMsgToTF..");
//    ROS_INFO_STREAM("orientation: " << latest_state.pose.pose.orientation.x << " "
//        << latest_state.pose.pose.orientation.y << " " << latest_state.pose.pose.orientation.z << " "
//        << latest_state.pose.pose.orientation.w);
    tf::quaternionMsgToTF(latest_state.pose.pose.orientation, temp_q);
//    ROS_INFO("done tf::quaternionMsgToTF.. ");
    btMatrix3x3 temp_mat = btMatrix3x3(temp_q);
    temp_mat.getEulerYPR(yaw, pitch, roll);
  }

  asctec_msgs::CtrlInput assemble_command(double roll, double pitch, double yaw_rate, int thrust, bool enable_roll,
                                          bool enable_pitch, bool enable_yaw, bool enable_thrust)
  {
    asctec_msgs::CtrlInput cin;
    cin.roll = -min(2047, max(-2047, int(roll * ROLL_SCALE))); // AscTec roll is opposite STARMAC
    cin.pitch = -min(2047, max(-2047, int(pitch * PITCH_SCALE))); // AscTec pitch is opposite STARMAC
    cin.yaw = -min(2047, max(-2047, int(yaw_rate * YAW_SCALE)));
    cin.thrust = min(max_thrust, max(0, int(double(thrust) * thrust_mult)));
    cin.ctrl = enable_pitch * CTRL_PITCH | enable_yaw * CTRL_YAW | enable_roll * CTRL_ROLL | enable_thrust
        * CTRL_THRUST;
    cin.chksum = cin.pitch + cin.roll + cin.yaw + cin.thrust + cin.ctrl + 0xAAAA;
    return cin;
  }

  void deadmanCallback(const ros::TimerEvent& e)
  {
    // This method, called periodically by a timer, checks for the various conditions that
    // result in a 'dead man' determination:
    // - motor_enable signal from teleop_flyer too old
    // - control inputs too old
    // - ll_status from asctec too old
    // - state estimate too old
    ros::Time now = ros::Time::now();
    //    double motor_enable_age = (now - last_motor_enable_time).toSec();
    //    double control_inputs_age = (now - last_control_input_time).toSec();
    //    double ll_status_age = (now - last_ll_status_time).toSec();
    //    double state_estimate_age = (now - last_state_time).toSec();
    string deadman_timeout_reason;
    ros::Duration deadman_timeout_dt(0);
    freq_status.tick();

    if (max_motor_enable_age > deadman_timeout)
    {
      deadman = true;
      deadman_timeout_reason = "motor enable";
      deadman_timeout_dt = max_motor_enable_age;
    }
    if (max_control_input_age > deadman_timeout)
    {
      deadman = true;
      deadman_timeout_reason = "control inputs";
      deadman_timeout_dt = max_control_input_age;
    }
    if (max_ll_status_age > deadman_timeout)
    {
      deadman = true;
      deadman_timeout_reason = "LL_Status";
      deadman_timeout_dt = max_ll_status_age;
    }
    if (max_state_age > deadman_timeout)
    {
      deadman = true;
      deadman_timeout_reason = "state";
      deadman_timeout_dt = max_state_age;
    }
    // Otherwise, if deadman conditions are no longer present,
    // and the ll_status (which must be recent if the above is true)
    // is indicating that the vehicle is not flying
    // (i.e. motors not spinning), then reset the deadman condition.

    else if (latest_ll_status.flying == 0)
    {
      deadman = false;
    }
    if (deadman)
    {
      ROS_ERROR_STREAM("Deadman timeout exceeded(" << deadman_timeout_reason << "), dt = "
          << deadman_timeout_dt.toSec());
    }
    diag_updater.update();
  }

  bool isSafeToReconfigure()
  {
    return (not last_motor_enable); // for now this should suffice
  }
};

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "starmac_asctec_adapter");
  starmac_robots_asctec::AscTecAdapter starmac_asctec_adapter;
  ros::spin();
  return 0;
}

