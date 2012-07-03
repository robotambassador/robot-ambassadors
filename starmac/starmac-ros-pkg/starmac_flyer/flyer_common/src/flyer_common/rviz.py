
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

import rospy
from numpy import array
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def ellipsoid_marker(position, size=[0.1, 0.1, 0.1], rgba=[1.0, 1.0, 1.0, 1.0], 
                     ns="", id=0, frame_id="", lifetime=None, stamp=None):
    marker = Marker()
    if stamp is None:
        marker.header.stamp = rospy.Time.now()
    else:
        marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.scale.x = size[0]
    marker.scale.y = size[1]
    marker.scale.z = size[2]
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    if lifetime is None:
        marker.lifetime = rospy.Duration()
    else:
        marker.lifetime = rospy.Duration(lifetime)
    return marker

def arrow_marker(position, direction, shaft_radius=0.02, head_radius=0.04, rgba=[1.0, 1.0, 1.0, 1.0], 
                     ns="", id=0, frame_id="", lifetime=None, stamp=None):
    marker = Marker()
    if stamp is None:
        marker.header.stamp = rospy.Time.now()
    else:
        marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    #marker.pose.position.x = position[0]
    #marker.pose.position.y = position[1]
    #marker.pose.position.z = position[2]
    marker.points = [Point(*tuple(position)), Point(*tuple(position+direction*1.0))]
    marker.scale.x = shaft_radius
    marker.scale.y = head_radius
    marker.scale.z = 1.0
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    if lifetime is None:
        marker.lifetime = rospy.Duration()
    else:
        marker.lifetime = rospy.Duration(lifetime)
    return marker

def mesh_marker(position, orientation=None, mesh_resource="", rgba=[1.0, 1.0, 1.0, 1.0],
                ns="", id=0, frame_id="", lifetime=None, stamp=None):
    marker = Marker()
    if stamp is None:
        marker.header.stamp = rospy.Time.now()
    else:
        marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = id
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = mesh_resource
    marker.action = Marker.ADD
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    if orientation is not None:
        marker.pose.orientation.x = orientation[0] 
        marker.pose.orientation.y = orientation[1] 
        marker.pose.orientation.z = orientation[2] 
        marker.pose.orientation.w = orientation[3] 
    #marker.points = [Point(*tuple(position)), Point(*tuple(position+direction*1.0))]
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    if lifetime is None:
        marker.lifetime = rospy.Duration()
    else:
        marker.lifetime = rospy.Duration(lifetime)
    return marker

def spherelist_marker(points, size=[0.1, 0.1, 0.1], rgba=[1.0, 1.0, 1.0, 1.0], 
                     ns="", id=0, frame_id="", lifetime=None, stamp=None):
    marker = Marker()
    if stamp is None:
        marker.header.stamp = rospy.Time.now()
    else:
        marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = id
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    for i in range(len(points)):
        marker.points.append(points[i])
    marker.scale.x = size[0]
    marker.scale.y = size[1]
    marker.scale.z = size[2]
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    if lifetime is None:
        marker.lifetime = rospy.Duration()
    else:
        marker.lifetime = rospy.Duration(lifetime)
    return marker

