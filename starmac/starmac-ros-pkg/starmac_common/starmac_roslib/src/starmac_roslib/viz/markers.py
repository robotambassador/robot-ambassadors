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
roslib.load_manifest('starmac_roslib')
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import rospy
import tf.transformations as tft
from math import radians, degrees, sin, cos
import numpy as np
from numpy.linalg import norm

from colors import Alpha, Colors, Alphas, OpaqueColors

default_frame_id = "/ned"

def stamp_now(msg, time=None):
    if time is None:
        msg.header.stamp = rospy.Time.now()
    else:
        msg.header.stamp = time
    return msg

class MarkerManager(object):
    """
    Class to manage marker id's
    """
    _id_counters = {}
    
    def get_ids(self, namespace, num_ids=1):
        return [self.get_id(namespace) for i in range(num_ids)]
    
    def get_id(self, namespace):
        if namespace in self._id_counters:
            self._id_counters[namespace] += 1
        else:
            self._id_counters[namespace] = 0
        return self._id_counters[namespace]
            
class MarkerBase(object):
    """
    Base class for various marker classes
    """
    # for storing the message(s) for later publication:
    _msg = None
    # common settings:
    _ns = ""
    _ids = []
    _frame_id = ""
    # other members:
    _visible = True
    _marker_manager = MarkerManager()
    _pub_marker = False
    
    def __init__(self, namespace="", num_ids=1, frame_id=None):
        self._ns = namespace
        self._ids = self._marker_manager.get_ids(self._ns, num_ids)
        if frame_id is not None:
            self._frame_id = frame_id
                    
    def publish(self):
        if self._pub_marker:
            self._publisher.publish(self._marker_msg)
            
    def _default_marker(self, index=0):
        m = Marker()
        m.ns = self._ns
        m.id = self._ids[index]
        m.header.frame_id = self._frame_id
        m.action = Marker.ADD
        m.frame_locked = True
        return m
            
    def get_msgs(self):
        raise NotImplementedError('get_msg must be implemented by derived class')

class MarkerGroup(object):
    
    def __init__(self, publisher):
        self._publisher = publisher
        self._markers = set()
        
    def add(self, *markers):
        for marker in markers:
            if not isinstance(marker, MarkerBase):
                raise TypeError('Can only add instances of ' + str(MarkerBase))
            self._markers.add(marker)
                
    def publish(self, time=None):
        marker_msgs = []
        for marker in self._markers:
            temp = marker.get_msgs()
            if temp is not None:
                thismarker_msgs = [stamp_now(m,time) for m in temp]
                marker_msgs.extend(thismarker_msgs)
        marker_array = MarkerArray(markers=marker_msgs)
        self._publisher.publish(marker_array)
            
class VerticalLineMarker(MarkerBase):
    def __init__(self, namespace, xypos, zstart=0, zend=1, frame_id=None, **kwargs):
        MarkerBase.__init__(self, namespace)
        self._xypos = xypos
        self._zstart = zstart
        self._zend = zend
        self._kwargs = kwargs
        if frame_id is None:
            self._frame_id = default_frame_id
        else:
            self._frame_id = frame_id
            
    def get_msgs(self):
        if self._msg is not None:
            return self._msg
        else:
            m = self._default_marker()
            m.type = Marker.LINE_STRIP
            m.scale.x = self._kwargs.get('width', 0.01)
            m.color.r, m.color.g, m.color.b, m.color.a = self._kwargs.get('color', OpaqueColors.WHITE)
            start_point = Point(self._xypos[0], self._xypos[1], self._zstart)
            end_point = Point(self._xypos[0], self._xypos[1], self._zend)
            m.points = [start_point, end_point] # will this work?
            self._msg = (m,)
        return self._msg
            
    def set(self, xypos=None, zstart=None, zend=None):
        if xypos is not None:
            self._xypos = xypos
        if zstart is not None:
            self._zstart = zstart
        if zend is not None:
            self._zend = zend
        self._msg = None # force message regen
            
class TextMarker(MarkerBase):
    def __init__(self, namespace, text, pos, frame_id=None, **kwargs):
        MarkerBase.__init__(self, namespace)
        self._kwargs = kwargs
        self._kwargs.update({'text': text, 'pos': pos})
        if frame_id is None:
            self._frame_id = default_frame_id
        else:
            self._frame_id = frame_id
            
    def get_msgs(self):
        if self._msg is not None:
            return self._msg
        else:
            m = self._default_marker()
            m.type = Marker.TEXT_VIEW_FACING
            m.scale.z = self._kwargs.get('height', 0.05)
            m.color.r, m.color.g, m.color.b, m.color.a = self._kwargs.get('color', OpaqueColors.WHITE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = self._kwargs['pos']
            m.text = self._kwargs['text']
            self._msg = m,
        return self._msg
    
    def set(self, **kwargs):
        self._kwargs.update(kwargs)
        self._msg = None #force update
        
class TrailMarker(MarkerBase):
    """
    There is already an rviz display for nav_msgs/Path messages, however it is not as
    flexible as this.
    """
    _max_points = 1000
    def __init__(self, namespace, points, colors=None, frame_id=None, **kwargs):
        MarkerBase.__init__(self, namespace)
        self._kwargs = kwargs
        self._kwargs.update({'points': points})
        self._kwargs.update({'colors': colors})
        if frame_id is None:
            self._frame_id = default_frame_id
        else:
            self._frame_id = frame_id
            
    def get_msgs(self):
        if self._msg is not None:
            return self._msg
        else:
            m = self._default_marker()
            m.type = Marker.LINE_STRIP
            m.scale.x = self._kwargs.get('width', 0.01)
            m.color.r, m.color.g, m.color.b, m.color.a = self._kwargs.get('color', OpaqueColors.WHITE)
            m.points = [Point(*p) for p in self._kwargs['points'][-self._max_points:]]
            colors = self._kwargs.get('colors')
            if colors is not None:
                m.colors = [ColorRGBA(*c) for c in colors[-self._max_points:]]
            self._msg = (m,)
        return self._msg
    
    def append_points(self, new_points, new_colors=None):
        self._kwargs['points'].extend(new_points)
        self._kwargs['points'] = self._kwargs['points'][-self._max_points:]
        colors = self._kwargs['colors']
        if colors is not None:
            if new_colors is not None:
                self._kwargs['colors'].extend(new_colors)
            else:
                self._kwargs['colors'].extend([self._kwargs['colors'][-1]]*len(new_points))
            self._kwargs['colors'] = self._kwargs['colors'][-self._max_points:]
        self._msg = None # force update
        
    def set_max_points(self, new_max):
        if new_max < self._max_points:
            self._kwargs['points'] = self._kwargs['points'][-new_max:]
            if self._kwargs['colors'] is not None:
                self._kwargs['colors'] = self._kwargs['colors'][-new_max:]
        self._max_points = new_max
        
class HeadingMarker(MarkerBase):
    """
    Actually 3 markers: a flat disc, an arrow and text. The text will show heading in degrees CW of +X,
    with a range of (-180,180].
    """
    def __init__(self, namespace, pos, heading=0, frame_id=None, **kwargs):
        MarkerBase.__init__(self, namespace, num_ids=3)
        self._kwargs = kwargs
        self._kwargs.update({'pos': pos})
        self._kwargs.update({'heading': heading})
        if frame_id is None:
            self._frame_id = default_frame_id
        else:
            self._frame_id = frame_id
    
    def get_msgs(self):
        if self._msg is not None:
            return self._msg
        else:
            pos = np.array(self._kwargs['pos'])
            length = self._kwargs.get('arrow_length', 0.15)
            heading = radians(self._kwargs['heading'])
            #disc
            mdisc = self._default_marker(0)
            mdisc.type = Marker.CYLINDER
            mdisc.pose.position.x, mdisc.pose.position.y, mdisc.pose.position.z = pos
            mdisc.scale.x = self._kwargs.get('radius', 0.1)*2
            mdisc.scale.y = self._kwargs.get('radius', 0.1)*2
            mdisc.scale.z = self._kwargs.get('height', 0.02)
            mdisc.color.r, mdisc.color.g, mdisc.color.b, mdisc.color.a = self._kwargs.get('color', Colors.WHITE + Alpha(0.3))
            #arrow
            marrow = self._default_marker(1)
            marrow.type = Marker.ARROW
            marrow.pose.position.x, marrow.pose.position.y, marrow.pose.position.z = self._kwargs['pos']
            marrow.scale.x = length
            marrow.scale.y = marrow.scale.z = self._kwargs.get('shaft_radius', 0.2)*2
            marrow.color.r, marrow.color.g, marrow.color.b, marrow.color.a = self._kwargs.get('color', Colors.RED + Alpha(0.8))
            mpo = marrow.pose.orientation
            quat = tft.quaternion_from_euler(heading, 0, 0, axes='rzyx')
            mpo.x, mpo.y, mpo.z, mpo.w = quat
            # text
            mtext = self._default_marker(2)
            mtext.type = Marker.TEXT_VIEW_FACING
            mtext.scale.z = self._kwargs.get('height', 0.05)
            mtext.color.r, mtext.color.g, mtext.color.b, mtext.color.a = self._kwargs.get('text_color', OpaqueColors.WHITE)
            offset = (length+0.1)*np.array([cos(heading), sin(heading) , 0])
            mtext.pose.position.x, mtext.pose.position.y, mtext.pose.position.z = pos + offset
            mtext.text = "%.1f" % degrees(heading)

            self._msg = (mdisc, marrow, mtext)
        return self._msg
    
    def set(self, **kwargs):
        self._kwargs.update(kwargs)
        self._msg = None #force update

class PolygonMarker(MarkerBase):
    def __init__(self, namespace, points, colors=None, frame_id=None, **kwargs):
        MarkerBase.__init__(self, namespace)
        self._kwargs = kwargs
        self._kwargs.update({'points': points})
        self._kwargs.update({'colors': colors})
        if frame_id is None:
            self._frame_id = default_frame_id
        else:
            self._frame_id = frame_id
            
    def get_msgs(self):
        if self._msg is not None:
            return self._msg
        else:
            m = self._default_marker()
            m.type = Marker.LINE_STRIP
            m.scale.x = self._kwargs.get('width', 0.01)
            m.color.r, m.color.g, m.color.b, m.color.a = self._kwargs.get('color', OpaqueColors.WHITE)
            m.points = [Point(*p) for p in self._kwargs['points']] + [Point(*self._kwargs['points'][0])]
            colors = self._kwargs.get('colors')
            if colors is not None:
                m.colors = [ColorRGBA(*c) for c in colors]
            self._msg = (m,)
        return self._msg
    
    def set(self, **kwargs):
        self._kwargs.update(kwargs)
        self._msg = None #force update
        
class PlanarRectangleWithRoundedEnds(PolygonMarker):
    """
    Useful for reach set overapproximations. Produces a polygonal marker that represents a rectangle of
    given length aligned with the given axis and with ends replaced by semicircles (TODO: better wording!)
     _______
    (_______)
      
    """
    def __init__(self, namespace, origin, axis, length, radius, frame_id=None, **kwargs):
        PolygonMarker.__init__(self, namespace, points=[], colors=None, frame_id=frame_id, **kwargs)
        self._kwargs = kwargs
        self._kwargs.update({'origin': origin, 'axis':axis, 'length':length, 'radius':radius})
        self._kwargs['points'] = self._get_points()
        if frame_id is None:
            self._frame_id = default_frame_id
        else:
            self._frame_id = frame_id

    def _get_points(self):
        origin = np.array(self._kwargs['origin'])
        axis = np.concatenate([self._kwargs['axis'], [0]])
        axis = axis/norm(axis)
        perp_axis = np.array([-axis[1], axis[0], 0])
        radius = self._kwargs['radius']
        length = self._kwargs['length']
        N = self._kwargs.get('n_segments', 20)
        semicirc1 = np.array([origin + radius*perp_axis*cos(angle) - radius*axis*sin(angle) for angle in np.linspace(0, np.pi, N)])
        origin2 = origin + axis*length
        semicirc2 = np.array([origin2 - radius*perp_axis*cos(angle) + radius*axis*sin(angle) for angle in np.linspace(0, np.pi, N)])
        points = np.concatenate([semicirc1, [origin - radius*perp_axis, origin2 - radius*perp_axis], 
                                 semicirc2, [origin2 + radius*perp_axis, origin + radius*perp_axis]])
        return points
        
    def set(self, **kwargs):
        self._kwargs.update(kwargs)
        self._kwargs['points'] = self._get_points()
        self._msg = None