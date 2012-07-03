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

using namespace flyer_controller::ControlModeTypes;

namespace flyer_controller
{
class ControlModeIdle : public ControlMode
{
private:
  control_mode_output control_out;

public:
  ControlModeIdle()
  {
  }

  void onInit()
  {
    ROS_INFO("ControlModeIdle onInit() called");
    ControlMode::onInit();
    requestRegistration("idle");
    control_out.control_mode = "idle";
    control_out.motors_on = false;
    control_out.yaw_rate_cmd = std::numeric_limits<double>::quiet_NaN();
  }

private:
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
        info = "";
      }
      else
      {
        ROS_ERROR("Invalid transition");
      }
    }
    else
    {
      ROS_ERROR_STREAM("Command unknown: " << msg->cmd);
    }
  }

  void outputControlTimerCallback(const ros::TimerEvent& e)
  {
    //ROS_INFO_STREAM(__PRETTY_FUNCTION__);
    switch (state)
    {
      case ControlModeTypes::STANDBY:
        output_pub.publish(control_out);
        break;
      case ControlModeTypes::ACTIVE:
        output_pub.publish(control_out);
        break;
      default:
        break;
    }
  }

}; // class
PLUGINLIB_DECLARE_CLASS(flyer_controller, ControlModeIdle, flyer_controller::ControlModeIdle, nodelet::Nodelet)
;

} // namespace

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "control_mode_idle");
//  flyer_controller::ControlModeIdle cmi;
//  ros::spin();
//  return 0;
//}
