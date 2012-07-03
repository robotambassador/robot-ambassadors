#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, UC Regents
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of California nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('starmac_kinect_obstacle_avoidance')
import rospy
import starmac_kinect.msg
import flyer_controller.msg
import joy.msg

class Demo(object):
    paused = False
    clear_time = None
    pause_cmd = flyer_controller.msg.control_mode_cmd('pause')
    proceed_cmd = flyer_controller.msg.control_mode_cmd('proceed')
    started = False
    
    def __init__(self):
        self.pub_cmd = rospy.Publisher("control_mode_autosequence/cmd", flyer_controller.msg.control_mode_cmd)
        self.sub_joy = rospy.Subscriber("teleop_flyer/joy", joy.msg.Joy, self.joy_cb)
        
    def joy_cb(self, joy_msg):
        if (not self.started) and (joy_msg.buttons[9] == 1):
            self.started = True
            rospy.loginfo("Starting demo..")
            self.sub_obstacle = rospy.Subscriber("kinect_estimator/obstacle", starmac_kinect.msg.Obstacle, self.obstacle_cb)
            
    def obstacle_cb(self, obs_msg):
        zpos = obs_msg.location.z
        obstacle = (obs_msg.obstacle_found and zpos < 1.0)
        rospy.logdebug('Got obstacle message, obstacle_found=%d obstacle z=%f' 
                      % (obs_msg.obstacle_found, zpos))
        # rotate obstacle location into body frame
        # if less than threshold, issue pause command
        if obstacle and not self.paused:
            rospy.loginfo('Obstacle at %f, Pausing...' % zpos)
            self.send_pause()
            self.paused = True
        # If no obstacles for 3 seconds, issue proceed command
        if self.paused:
            if self.clear_time is None:
                self.clear_time = rospy.Time.now()
            else:
                if (not obstacle) and \
                        (rospy.Time.now() - self.clear_time).to_sec() > 3.0:
                    rospy.loginfo('Proceed..')
                    self.send_proceed()
                    self.clear_time = None
                    self.paused = False
        
    def send_pause(self):
        rospy.loginfo('Sending PAUSE command')
        self.pub_cmd.publish(self.pause_cmd)
        
    def send_proceed(self):
        rospy.loginfo('Sending PROCEED command')
        self.pub_cmd.publish(self.proceed_cmd)
        
    def spin(self):
        rospy.spin()
            
if __name__ == "__main__":
    rospy.init_node('kinect_obstacle_avoidance_demo')
    self = Demo()
    self.spin()