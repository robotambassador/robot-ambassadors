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
#include <joy/Joy.h>
#include <math.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sys/select.h>
class JoyDummy
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;

  ros::Publisher joy_pub;

  ros::Timer publish_timer;

  joy::Joy joy_msg;

  // Parameters
  double publish_rate; // [Hz] rate to republish Joy messages at
  double twiddle_period; // [s] period for 'twiddling' each axis
  std::string mode;
  bool go_operational;

  // Member vars
  ros::Time start_time;
  int kfd;
  struct termios cooked, raw;

public:
  JoyDummy() :
    nh_priv("~"), publish_rate(20), twiddle_period(2.0), mode("twiddle"), kfd(0)
  {
    nh_priv.param("publish_rate", publish_rate, publish_rate);
    nh_priv.param("twiddle_period", twiddle_period, twiddle_period);
    nh_priv.param("mode", mode, mode);
    nh_priv.param("go_operational", go_operational, go_operational);
    joy_pub = nh.advertise<joy::Joy> ("joy", 1);

    publish_timer = nh.createTimer(ros::Duration(1 / publish_rate), &JoyDummy::publishJoyCallback, this);

    joy_msg.axes.resize(10, 0);
    joy_msg.buttons.resize(10, 0);

    start_time = ros::Time::now();

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Number keys 1-9 map to joystick buttons");
    puts("Number key 0 toggles trigger");

  }

  ~JoyDummy()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kbhit()
  {
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_SET(kfd, &fds);
    return select(1, &fds, NULL, NULL, &tv);
  }
  int getch()
  {
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0)
    {
      return r;
    }
    else
    {
      return c;
    }
  }
  void publishJoyCallback(const ros::TimerEvent& e)
  {
    double t = (ros::Time::now() - start_time).toSec();
    double x;
    int ax;
    static double t_confirm = 0;
    static bool kb_trigger = false;

    joy_msg.axes.assign(10, 0);
    joy_msg.buttons.assign(10, 0);

    int c;
    if (kbhit())
    {
      c = getch();
      if (c > 48 and c <= 57)
      {
        joy_msg.buttons[c - 48] = 1;
        ROS_INFO_STREAM("Setting button: " << (c-48));
      }
      else if (c == 48)
      {
        kb_trigger = !kb_trigger;
        ROS_INFO_STREAM("Setting trigger: " << int(kb_trigger));
      }
      else
      {
        ROS_INFO_STREAM("Got character code: " << c);
      }
    }
    joy_msg.buttons[0] = int(kb_trigger);

    if (mode == "twiddle")
    {
      x = sin(t / twiddle_period * M_PI * 2);
      ax = int(floor(t / twiddle_period)) % 4;
      joy_msg.axes[3] = -1.0; // force throttle to end up low
      joy_msg.axes[ax] = x;
    }
    if (go_operational)
    {
      // Command go to operational:
      if (5.0 <= t && t <= 5.2)
      {
        joy_msg.buttons[2] = 1;
        t_confirm = 6.0;
      }

      // Command attitude mode:
      if (7.0 <= t && t <= 7.2)
      {
        joy_msg.buttons[2] = 1;
        t_confirm = 8.0;
      }

      // Confirm commands:
      if (t_confirm <= t && t <= t_confirm + 0.2)
      {
        joy_msg.buttons[8] = 1;
      }
      else if (t > t_confirm + 0.2)
      {
        t_confirm = 0;
      }

      if (t >= 10.0 && t < 20.0)
      {
        joy_msg.buttons[0] = 1;
      }
      else if (t >= 20.0)
      {
        joy_msg.buttons[0] = 0;
      }
    }
    joy::Joy joy_copy = joy_msg;
    joy_pub.publish(joy_copy);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_dummy");
  JoyDummy joy_dummy;

  ros::spin();

  return 0;
}
