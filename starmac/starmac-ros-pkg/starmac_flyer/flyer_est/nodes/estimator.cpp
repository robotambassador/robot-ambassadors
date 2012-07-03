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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <string>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

using namespace std;

namespace flyer_est
{

// Constants for velocity filter:
const double VEL_FILT_A = 0.9;
const double VEL_FILT_B = 0.1;

class Estimator : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;
  double min_freq;
  double max_freq;
  diagnostic_updater::FrequencyStatus freq_status;
  // Parameters
  string transform_topic;
  double freq;
  double xy_vel_filt_a;
  double xy_vel_filt_b;
  double z_vel_filt_a;
  double z_vel_filt_b;
  // Publishers
  ros::Publisher output_pub;
  // Subscribers
  ros::Subscriber transform_sub;
  // TF
  tf::TransformListener tf_listener;
  // Timers
  ros::Timer main_timer;
  // Members
  string flyer_imu_tf_node;
  bool got_first_transform;
  double last_transform_age;
  tf::StampedTransform last_transform;
  tf::StampedTransform T_flyer_vicon__flyer_imu; // Transform from flyer_vicon to flyer_imu, grabbed at startup
  tf::StampedTransform T_enu__ned; // Transform from /enu to /ned, grabbed at startup
  nav_msgs::Odometry state_msg;
  ros::Timer get_transforms_timer;

public:
  Estimator() :
    diag_updater(), min_freq(40), max_freq(60),
        freq_status(diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq)),
        transform_topic("vicon_recv_direct/output"), freq(50), xy_vel_filt_a(0.9), xy_vel_filt_b(0.1),
        z_vel_filt_a(0.9), z_vel_filt_b(0.1), got_first_transform(false)
  {
  }

  void onInit()
  {
    nh = getMTNodeHandle();
    nh_priv = getMTPrivateNodeHandle();

    // Diagnostics
    diag_updater.add("Estimator Status", this, &Estimator::diagnostics);
    diag_updater.add(freq_status);
    diag_updater.setHardwareID("none");
    diag_updater.force_update();
    // Parameters
    nh_priv.param("transform_topic", transform_topic, transform_topic);
    nh_priv.param("freq", freq, freq);
    nh_priv.param("xy_vel_filt_a", xy_vel_filt_a, xy_vel_filt_a);
    nh_priv.param("xy_vel_filt_b", xy_vel_filt_b, xy_vel_filt_b);
    nh_priv.param("z_vel_filt_a", z_vel_filt_a, z_vel_filt_a);
    nh_priv.param("z_vel_filt_b", z_vel_filt_b, z_vel_filt_b);
    min_freq = freq * 0.9;
    max_freq = freq * 1.1;
    // Publishers
    output_pub = nh_priv.advertise<nav_msgs::Odometry> ("output", 10);
    // Get transform from flyer_vicon to flyer_imu
    flyer_imu_tf_node = ros::this_node::getNamespace() + "/flyer_imu";
    get_transforms_timer = nh.createTimer(ros::Duration(0.01), boost::bind(&Estimator::getTransforms, this, _1), true);
  }

  void getTransforms(const ros::TimerEvent& event)
  {
    string flyer_vicon_tf_node = ros::this_node::getNamespace() + "/flyer_vicon";
    ROS_INFO("Looking up flyer_vicon --> flyer_imu transform");
    bool gotit = false;
    ros::Duration waiter(0.5);
    while (!gotit)
    {
      try
      {
        tf_listener.lookupTransform(flyer_vicon_tf_node, flyer_imu_tf_node, ros::Time(0), T_flyer_vicon__flyer_imu);
        tf_listener.lookupTransform("/enu", "/ned", ros::Time(0), T_enu__ned);
        gotit = true;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("Exception looking up transform: %s",ex.what());
      }
      waiter.sleep();
    }
    ROS_INFO_STREAM("Got flyer_vicon --> flyer_imu transform: " << T_flyer_vicon__flyer_imu.getOrigin().getX()
        << " " << T_flyer_vicon__flyer_imu.getOrigin().getY()
        << " " << T_flyer_vicon__flyer_imu.getOrigin().getZ());
    ROS_INFO_STREAM("Got /enu --> /ned transform: " << T_enu__ned.getOrigin().getX()
        << " " << T_enu__ned.getOrigin().getY()
        << " " << T_enu__ned.getOrigin().getZ());
    // Subscribers
    transform_sub = nh.subscribe(transform_topic, 1, &Estimator::transformCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    // Timers
    main_timer = nh.createTimer(ros::Duration(1 / freq), &Estimator::mainCallback, this);
  }

private:
  void transformCallback(const geometry_msgs::TransformStampedConstPtr& msg)
  {
    // Ok this is a bit convoluted. What is happening here:
    // - we receive a stamped transform providing the transform from /enu to flyer_vicon
    // - what we want is to output a nav_msgs/Odometry (position, orientation, velocity, ang vel)
    //   (though we'll leave ang vel zero for now, as well as the covariances) from /ned to flyer_imu
    // - and also to broadcast the position, orientation transform to /tf
    // - tf knows the transform between flyer_imu and flyer_vicon
    //  -- this doesn't change during a run -- so this is something we should just query at startup, store,
    //     and be done with it.
    // Steps:
    // - convert the received transform message to a tf::StampedTransform
    // - come up with the overall transform: /ned --> /enu --> flyer_vicon --> flyer_imu
    // - calculate filtered velocity
    // - combine transform and velocity into Odometry message, ready for publication at the next
    //   timer interval
    // TODO: somewhere, the /enu --> flyer_vicon relationship should be broadcast to TF so that visualization
    //       etc works.
    static ros::Time prev_stamp = ros::Time::now();
    tf::StampedTransform T_enu__flyer_vicon;
    tf::transformStampedMsgToTF(*msg, T_enu__flyer_vicon);
    tf::StampedTransform T_ned_imu;
    T_ned_imu.setData(T_enu__ned.inverse() * T_enu__flyer_vicon * T_flyer_vicon__flyer_imu);
    T_ned_imu.stamp_ = (*msg).header.stamp; // make sure the stamp is that of the data we just got
//    ROS_INFO_STREAM("/ned --> /imu transform: " << T_ned_imu.getOrigin().getX()
//        << " " << T_ned_imu.getOrigin().getY()
//        << " " << T_ned_imu.getOrigin().getZ());

    // Now do velocity calculations
    double dt = (T_ned_imu.stamp_ - prev_stamp).toSec();
    if (dt > 0.001)
    {
      tf::Vector3 trans = T_ned_imu.getOrigin();
      state_msg.pose.pose.position.x = trans[0];
      state_msg.pose.pose.position.y = trans[1];
      state_msg.pose.pose.position.z = trans[2];
      tf::Quaternion quat = T_ned_imu.getRotation();
      state_msg.pose.pose.orientation.w = quat.getW();
      state_msg.pose.pose.orientation.x = quat.getX();
      state_msg.pose.pose.orientation.y = quat.getY();
      state_msg.pose.pose.orientation.z = quat.getZ();
      freq_status.tick();
      calcVelocity(state_msg.pose.pose.position.x, state_msg.pose.pose.position.y, state_msg.pose.pose.position.z, dt,
                   state_msg.twist.twist.linear.x, state_msg.twist.twist.linear.y, state_msg.twist.twist.linear.z);
      state_msg.header.stamp = T_ned_imu.stamp_;
    }
    else
    {
      ROS_WARN_STREAM("dt too small: " << dt);
    }

    prev_stamp = T_ned_imu.stamp_;
    if (!got_first_transform)
      got_first_transform = true;
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    stat.add("Got TF data", got_first_transform);
    stat.add("Last transform age", last_transform_age);
  }

  void calcVelocity(const double& current_x, const double& current_y, const double& current_z, const double dt,
                    double& new_vx, double& new_vy, double& new_vz)
  {
    static double prev_x, prev_y, prev_z;
    static double prev_vx = 0, prev_vy = 0, prev_vz = 0;
    static bool first = true;
    if (first)
    {
      new_vx = new_vy = new_vz = 0;
      first = false;
    }
    else
    {
      if (dt > 0.0001)
      {
        new_vx = xy_vel_filt_a * (current_x - prev_x) / dt + xy_vel_filt_b * prev_vx;
        new_vy = xy_vel_filt_a * (current_y - prev_y) / dt + xy_vel_filt_b * prev_vy;
        new_vz = z_vel_filt_a * (current_z - prev_z) / dt + z_vel_filt_b * prev_vz;
      }
      else
      {
        ROS_WARN_STREAM("dt too small: " << dt);
        new_vx = prev_vx;
        new_vy = prev_vy;
        new_vz = prev_vz;
      }
    }
    prev_x = current_x;
    prev_y = current_y;
    prev_z = current_z;
    prev_vx = new_vx;
    prev_vy = new_vy;
    prev_vz = new_vz;

  }

  void mainCallback(const ros::TimerEvent& e)
  {
    static bool first = true;
    if (!got_first_transform)
    {
      if (first)
      {
        ROS_INFO_STREAM("Waiting for transforms from /ned to " << flyer_imu_tf_node);
        first = false;
      }
    }
    else
    {
      nav_msgs::OdometryPtr state_msg_ptr(new nav_msgs::Odometry);
      *state_msg_ptr = state_msg;
      output_pub.publish(state_msg_ptr);
    }

    diag_updater.update();

  }
};
PLUGINLIB_DECLARE_CLASS(flyer_est, Estimator, flyer_est::Estimator, nodelet::Nodelet)
;

} // namespace

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "estimator");
//  flyer_est::Estimator estimator;
//  ros::spin();
//  return 0;
//}

