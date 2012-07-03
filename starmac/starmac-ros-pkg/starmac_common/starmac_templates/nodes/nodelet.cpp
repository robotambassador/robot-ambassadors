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
#include <std_msgs/String.h>

namespace some_namespace
{

using std::string;

class MyNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_priv;
  ros::NodeHandle nh;
  // Params
  bool enable_awesomeness; // enable cool stuff
  double awesome_factor; // [rad/s] amount of awesomeness
  // Publishers
  ros::Publisher awesome_pub;
  // Subscribers
  ros::Subscriber chatter_sub;
  // Timers
  ros::Timer output_timer;
  // Members
  string latest_chatter;

public:
  MyNodelet() :
    enable_awesomeness(true), awesome_factor(1.57)
  {
  }
private:
  void onInit()
  {
    nh = getMTNodeHandle();
    nh_priv = getMTPrivateNodeHandle();

    NODELET_INFO("onInit()");

    // Parameters
    nh_priv.param("enable_awesomeness", enable_awesomeness, enable_awesomeness);
    nh_priv.param("awesome_factor", awesome_factor, awesome_factor);
    // Publishers
    awesome_pub = nh_priv.advertise<std_msgs::String> ("awesome", 10);
    // Subscribers
    chatter_sub = nh.subscribe("chatter", 10, &MyNodelet::chatterCb, this);
    // Timers
    output_timer = nh.createTimer(ros::Duration(0.1), &MyNodelet::outputTimerCb, this);
  }

  void outputTimerCb(const ros::TimerEvent& e)
  {
    std_msgs::String output;
    std::ostringstream s;
    s << "foo bar baz " << awesome_factor;
    output.data = s.str();
    awesome_pub.publish(output);
  }

  void chatterCb(const std_msgs::StringConstPtr& chatter)
  {
    NODELET_INFO_STREAM("Heard chatter: " << chatter->data);
  }

};
PLUGINLIB_DECLARE_CLASS(some_namespace, MyNodelet, some_namespace::MyNodelet, nodelet::Nodelet)
;
}
