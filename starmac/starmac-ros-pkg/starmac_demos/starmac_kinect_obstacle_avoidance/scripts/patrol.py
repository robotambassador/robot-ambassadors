#!/usr/bin/env rosh

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
patrol.py

Set up an autosequence for the vehicle to follow as part of the obstacle avoidance demo.
"""

import roslib
roslib.load_manifest('starmac_kinect_obstacle_avoidance')

from starmac_roshlib.topics import is_publishing, wait_for_topic
from starmac_roshlib.commanding import cmd_wait

def runscript(script):
    for line in script.split('\n'):
        line = line.strip()
        if len(line) > 0:
            dest, cmd = line.split(' ', 1)
            topic_obj = {'CTRL':ccmd, 'AUTO':acmd}[dest]
            cmd_wait(topic_obj,cmd)

wait_for_topic('/pelican1/manager','/pelican1/controller/cmd')
wait_for_topic('/pelican1/manager','/pelican1/control_mode_autosequence/cmd')
sleep(10.0)

ccmd = topics.pelican1.controller.cmd
acmd = topics.pelican1.control_mode_autosequence.cmd

cmd_wait(ccmd, 'mode operational', 3.0)
script1 = """\
CTRL control_mode to_active autosequence
"""
script2 = """\
AUTO foo
AUTO bar
AUTO baz
AUTO define hover_point A_north 0.800000 0.900000 0 0 45
AUTO define hover_point A_east 0.800000 0.900000 0 0 135
AUTO define hover_point A_south 0.800000 0.900000 0 0 225
AUTO define hover_point A_west 0.800000 0.900000 0 0 315

AUTO define hover_point B_north 0.800000 -0.600000 0 0 45
AUTO define hover_point B_east 0.800000 -0.600000 0 0 135
AUTO define hover_point B_south 0.800000 -0.600000 0 0 225
AUTO define hover_point B_west 0.800000 -0.600000 0 0 315

AUTO define hover_point C_north -0.800000 -0.600000 0 0 45
AUTO define hover_point C_east -0.800000 -0.600000 0 0 135
AUTO define hover_point C_south -0.800000 -0.600000 0 0 225
AUTO define hover_point C_west -0.800000 -0.600000 0 0 315

AUTO define hover_point D_north -0.800000 0.900000 0 0 45
AUTO define hover_point D_east -0.800000 0.900000 0 0 135
AUTO define hover_point D_south -0.800000 0.900000 0 0 225
AUTO define hover_point D_west -0.800000 0.900000 0 0 315

AUTO define autosequence patrol_square A_west pause B_west B_south C_south C_east D_east D_north A_north A_west B_west B_south C_south C_east D_east D_north A_north A_west B_west B_south C_south C_east D_east D_north A_north A_west B_west B_south C_south C_east D_east D_north A_north A_west
AUTO execute autosequence patrol_square
"""
#AUTO define autosequence patrol A_north A_west pause B_west B_north B_east A_east A_north A_west B_west B_north B_east A_east A_north A_west B_west B_north B_east A_east A_north A_west B_west B_north B_east A_east A_north A_west B_west B_north B_east A_east A_north A_west B_west B_north B_east A_east A_north A_west

runscript(script1)
sleep(3.0)
runscript(script2)
        
loginfo('My work here is done..')