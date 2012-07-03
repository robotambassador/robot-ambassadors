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
#ifndef FLYER_CONTROLLER_CONTROL_UTILS_H
#define FLYER_CONTROLLER_CONTROL_UTILS_H

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

inline void odom_msg_to_xyz(const nav_msgs::OdometryConstPtr& odom_msg, double& x, double& y, double& z)
{
  x = odom_msg->pose.pose.position.x;
  y = odom_msg->pose.pose.position.y;
  z = odom_msg->pose.pose.position.z;
}

inline void odom_msg_to_lin_vel(const nav_msgs::OdometryConstPtr& odom_msg, double& vx, double& vy, double& vz)
{
  vx = odom_msg->twist.twist.linear.x;
  vy = odom_msg->twist.twist.linear.y;
  vz = odom_msg->twist.twist.linear.z;
}

inline void odom_msg_to_ypr(const nav_msgs::OdometryConstPtr& odom_msg, double& yaw, double& pitch, double& roll)
{
  tf::Quaternion temp_q;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, temp_q);
  btMatrix3x3 temp_mat = btMatrix3x3(temp_q);
  temp_mat.getEulerYPR(yaw, pitch, roll);
}

#endif // FLYER_CONTROLLER_CONTROL_UTILS_H
