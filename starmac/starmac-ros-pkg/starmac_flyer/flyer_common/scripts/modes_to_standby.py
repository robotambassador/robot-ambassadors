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
"""
modes_to_standby.py

This roshlet is used to bring additional control modes to standby. It also necessarily brings the
controller to operational first.

Usage:
rosrun rosh roshlet.py flyer_common/modes_to_standby.py mode_1 [mode_2 [... mode_n]] --plugins=rosh_common,rosh_geometry
"""
import sys
import roslib
roslib.load_manifest('flyer_common')
from rospy import myargv, get_namespace, loginfo
import rospy

from starmac_roshlib.topics import is_publishing, wait_for_topic
from starmac_roshlib.commanding import cmd_wait

#rospy.init_node('modes_to_standby')

loginfo('modes_to_standby script starting')

ns = get_namespace()
print "ns = ", ns

modes = rospy.get_param('~modes').split() #parameters['~'].modes() #myargv(argv=sys.argv)
loginfo('modes: %s' % modes)

wait_for_topic(ns + 'manager', ns + 'controller/cmd')
for mode in modes:
    wait_for_topic(ns + 'manager', ns + 'control_mode_%s/cmd' % mode)
sleep(5.0)

ccmd = topics[ns + 'controller/cmd']

cmd_wait(ccmd, "mode operational", 3.0)

for mode in modes:
    cmd_wait(ccmd, "control_mode to_standby %s" % mode, 2.0)
    
# TODO: add some verification that the modes actually end up in standby
loginfo('modes_to_standby script finished')
