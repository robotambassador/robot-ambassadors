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
roslib.load_manifest('starmac_templates') # change this to your package name
import rospy
from starmac_roslib.timers import Timer
# Message imports:
# Make sure corresponding packages are listed as dependencies in package manifest!
from std_msgs.msg import Bool, Float64

class PythonNodeExample(object):
    """
    Example python node including reading parameters, publishers and subscribers, and timer.
    """
    def __init__(self):
        # Remember that init_node hasn't been called yet so only do stuff here that
        # doesn't require node handles etc.
        pass

    def start(self):
        rospy.init_node('python_node_example')
        self.init_params()
        self.init_vars()
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()
        rospy.spin()
        
    def init_params(self):
        self.param_one = rospy.get_param("~param_one", 100.0)
        self.param_two = rospy.get_param("~param_two", False)
    
    def init_vars(self):
        self.some_variable = 42.0
        self.latest_input = None

    def init_publishers(self):
        self.bool_topic_pub = rospy.Publisher('bool_topic', Bool)
        
    def init_subscribers(self):
        self.some_input_sub = rospy.Subscriber('input_topic', Float64, self.some_input_callback)
      
    def init_timers(self):
        self.heartbeat_timer = Timer(rospy.Duration(1/10.0), self.heartbeat_timer_callback) # Call at 10 Hz
                                      
    def some_input_callback(self, msg):
        rospy.logdebug('Got input: ' + str(msg))
        self.latest_input = msg
        if self.latest_input.data > self.param_one:
            self.bool_topic_pub.publish(Bool(True))
        else:
            self.bool_topic_pub.publish(Bool(False))
    
    def heartbeat_timer_callback(self, event):
        if event.last_real is not None:
            dt = (event.current_real - event.last_real).to_sec()
            rospy.loginfo('Heartbeat. dt = %f' % dt)
        
        
if __name__ == "__main__":
    self = PythonNodeExample()
    self.start()