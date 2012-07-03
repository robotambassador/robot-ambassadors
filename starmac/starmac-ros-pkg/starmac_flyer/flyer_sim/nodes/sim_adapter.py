#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, UC Regents
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

import roslib
roslib.load_manifest('flyer_sim')
import rospy
import numpy as np
from math import sin, cos, radians, degrees
import tf.transformations as tft

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from flyer_controller.msg import control_mode_output

from starmac_roslib.timers import Timer
from starmac_roslib.pid import PidController

from flyer_sim.models.simple_nonlinear import SimpleNonlinearModel
from flyer_sim.models.linear import LinearModel

class SimAdapter(object):

    def __init__(self):
        pass
        
    def start(self):
        rospy.init_node('sim_adapter')
        self.init_params()
        self.init_state()
        self.init_vars()
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()
        rospy.spin()
        
    def init_vars(self):
        self.latest_cmd_msg = control_mode_output()
        self.motor_enable = False
        self.thrust_cmd = 0.0
        
    def init_state(self):
        if self.model == "simple_nonlinear":
            self.model = SimpleNonlinearModel()
        elif self.model == "linear":
            self.model = LinearModel()
        else:
            rospy.logerror("Model type '%s' unknown" % self.model)
            raise Exception("Model type '%s' unknown" % self.model)
        
    def init_params(self):
        self.model = rospy.get_param("~model", "simple_nonlinear")

    def init_publishers(self):
        # Publishers
        self.pub_odom = rospy.Publisher('odom', Odometry)
        
    def init_subscribers(self):
        # Subscribers
        self.control_input_sub = rospy.Subscriber('controller_mux/output', control_mode_output, self.control_input_callback)
        self.motor_enable_sub = rospy.Subscriber('teleop_flyer/motor_enable', Bool, self.motor_enable_callback)
      
    def init_timers(self):
        self.simulation_timer = Timer(rospy.Duration(1/50.0), self.simulation_timer_callback)
                                      
    # Subscriber callbacks:
    def control_input_callback(self, msg):
        rospy.logdebug('Current command is: ' + str(msg))
        self.latest_cmd_msg = msg
    
    def motor_enable_callback(self, msg):
        if msg.data != self.motor_enable:
            #rospy.loginfo('Motor enable: ' + str(msg.data))
            self.motor_enable = msg.data
    
    # Timer callbacks:
    def simulation_timer_callback(self, event):
        if False:
            print 'last_expected:        ', event.last_expected
            print 'last_real:            ', event.last_real
            print 'current_expected:     ', event.current_expected
            print 'current_real:         ', event.current_real
            print 'current_error:        ', (event.current_real - event.current_expected).to_sec()
            print 'profile.last_duration:', event.last_duration.to_sec()
            if event.last_real:
                print 'last_error:           ', (event.last_real - event.last_expected).to_sec(), 'secs'
            print
        if event.last_real is None:
            dt = 0.0
        else:
            dt = (event.current_real - event.last_real).to_sec()
            self.update_controller(dt)
            self.update_state(dt)
        #rospy.loginfo("position: " + str(self.position) + " velocity: " + str(self.velocity) + " thrust_cmd: " + str(self.thrust_cmd))
        self.publish_odometry()
        
    def update_state(self, dt):
        # The following model is completely arbitrary and should not be taken to be representative of
        # real vehicle performance!
        # But, it should be good enough to test out control modes etc.
        self.model.update(dt)
            
    def update_controller(self, dt):
        lcm = self.latest_cmd_msg
        self.model.set_input(np.array([lcm.yaw_cmd, lcm.pitch_cmd, lcm.roll_cmd, lcm.alt_cmd, lcm.motors_on]), dt)
        #rospy.loginfo("thrust_cmd = %f, dt = %f" % (self.thrust_cmd, dt))
                    
    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "/ned"
        oppp = odom_msg.pose.pose.position
        oppp.x, oppp.y, oppp.z  = self.model.get_position()
        ottl = odom_msg.twist.twist.linear
        ottl.x, ottl.y, ottl.z = self.model.get_velocity()
        oppo = odom_msg.pose.pose.orientation
        oppo.x, oppo.y, oppo.z, oppo.w = self.model.get_orientation()
        otta = odom_msg.twist.twist.angular
        otta.x, otta.y, otta.z = self.model.get_angular_velocity()
        self.pub_odom.publish(odom_msg)
        
      
if __name__ == "__main__":
  self = SimAdapter()
  self.start()