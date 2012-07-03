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

import math
import random
import numpy as np
from numpy.linalg import norm

import roslib
roslib.load_manifest('starmac_roslib')
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import rospy

from starmac_roslib.viz.colors import Alpha, Colors
from starmac_roslib.viz.markers import MarkerGroup, VerticalLineMarker, TextMarker, \
    TrailMarker, HeadingMarker, PolygonMarker, PlanarRectangleWithRoundedEnds

def batt_color(voltage):
    if voltage > 10.5:
        return Colors.GREEN
    elif voltage >= 10.0:
        return Colors.YELLOW
    else:
        return Colors.RED
    
rospy.init_node('markers_test')
mpub = rospy.Publisher('markers', Marker)
mapub = rospy.Publisher('markers_array', MarkerArray)
mg = MarkerGroup(mapub)
default_frame_id = "/ned"
vlm = VerticalLineMarker('demo', (1,1), color=Colors.WHITE + Alpha(0.5))
tm = TextMarker('demo', 'foo', (0,0,0))
tm2 = TextMarker('demo', 'Hello', (0,0,0))
batt_txt = TextMarker('battery', '', (0,0,0))
trail = TrailMarker('trail', [], colors=[], color=Colors.BLUE + Alpha(0.8))
trail.set_max_points(500)
ground_trail = TrailMarker('ground_track', [], color=Colors.GREEN + Alpha(0.2))
ground_nominal_trail = TrailMarker('commanded', [], color=Colors.YELLOW + Alpha(0.2), width=0.015)
heading = HeadingMarker('heading', (0,0,0), height=0.02)
boundary = PolygonMarker('boundary', ((1,1,0), (-1,1,0), (-1,-1,0), (1,-1,0)), color=Colors.RED+Alpha(0.5), width=0.02)
prre = PlanarRectangleWithRoundedEnds('prre', (0,0,0), (1,0), 5, 0.25, color=Colors.GREEN+Alpha(0.8))
mg.add(vlm, tm, tm2, batt_txt, trail, ground_trail, ground_nominal_trail, heading, boundary, prre)
r = rospy.Rate(50)
angle = 0
points = []
print "Run the following command:"
print "rosrun rviz rviz -d $(rospack find starmac_roslib)/demos/testviz.vcg"
last_time = rospy.Time.now()
last_pub_time = rospy.Time.now()
x, y, z, vx, vy, vz = 0, 0, 1, 0, 0, 0
xerr, yerr = 0, 0
vxerr, vyerr = 0, 0 
voltage = 12.6
while not rospy.is_shutdown():
    this_time = rospy.Time.now()
    dt = (this_time - last_time).to_sec()
    last_time = this_time
    ax, ay, az = (random.gauss(0.05,0.5)*math.cos(angle)+random.gauss(0,0.05) - 0.1*x, 
        random.gauss(0.1,0.5)*math.sin(angle)+random.gauss(0,0.1)-0.1*y, 
        random.gauss(0,0.5))
    vx, vy, vz = vx + ax*dt, vy + ay*dt, vz + az*dt
    vxerr, vyerr = random.gauss(0,0.1), random.gauss(0,0.1)
    xerr, yerr = xerr + vxerr*dt, yerr + vyerr*dt
    if z <= 0.0 and vz < 0:
        vz = 0
    elif z >= 1.5 and vz > 0:
        vz = 0
    x, y, z = min(10,max(-10,x + vx*dt )), min(10,max(-10,y + vy*dt )), min(1.5,max(0,z + vz*dt)) 
    xout,yout = x + xerr, y+ yerr
    alt = z + random.gauss(0,0.001)
    voltage = max(9.0, voltage - dt*3/(3*60))
    rospy.logdebug('%f, %f, %f' % (xout, yout, alt))
    trail.append_points([(xout,yout,alt)], [(min(1,1.0*alt/2.0), 1-min(1,1.0*alt/1.0), 1-min(1,alt/0.8), 0.9)])
    ground_trail.append_points([(xout,yout,0)])
    ground_nominal_trail.append_points([(x,y,0)])
    vlm.set((xout,yout), zend=alt)
    tm.set(text="%.2f,%.2f" % (xout,yout), pos=(xout,yout,-0.02))
    tm2.set(text="%.2f" % alt, pos=(xout+0.1,yout,alt/2))
    batt_txt.set(text="%.2f V" % voltage, color=batt_color(voltage) + Alpha(1.0), pos=(xout,yout,alt+0.1))
    heading.set(pos=(xout,yout,z+0.01), heading=math.degrees(math.atan2(vy,vx)))
    prre.set(origin=(xout,yout,0), axis=(vx,vy), length=norm(np.array([vx,vy])))
    dt_pub = (this_time - last_pub_time).to_sec()
    if dt_pub > 0.05:
        mg.publish()
        last_pub_time = this_time
    angle += math.radians(0.5) % math.radians(360)
    r.sleep()