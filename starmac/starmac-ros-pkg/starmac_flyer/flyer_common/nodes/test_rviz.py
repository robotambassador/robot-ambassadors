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

import roslib; roslib.load_manifest('flyer_common')
import rospy
from visualization_msgs.msg import Marker
from flyer_common.rviz import ellipsoid_marker, mesh_marker
from tf.transformations import quaternion_from_euler
from math import radians

class TestRviz:
    def __init__(self):
        rospy.init_node('test_rviz')
        self.marker_pub = rospy.Publisher("test_markers", Marker)
        
    def test_markers(self):
#        test_ellipsoid = ellipsoid_marker([0,0,0], frame_id="/map")
#        self.marker_pub.publish(test_ellipsoid)
        quat = quaternion_from_euler(radians(-90),radians(-90),0,'rxyz')
        test_mesh = mesh_marker([1,0,0], orientation=quat, mesh_resource="file:///tmp/hybridlab.dae", frame_id="/map",
                                rgba=[1,1,1,0.5])
        self.marker_pub.publish(test_mesh)
        
    def go(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.test_markers()
            r.sleep()
    
if __name__ == "__main__":
    self = TestRviz()
    self.go()