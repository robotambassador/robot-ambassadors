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

from model_base import ModelBase

model_select = 2

if model_select == 1:
    # Data from Anil's email "Re: Did some simulations", Wed, Feb 9, 2011 at 8:30 PM 
    A = np.array([  
        [1.0000,       0,  0.0004,  0.0212,  0.0043, -0.0034,  0.0001,  0.0003,  0.0008,  0.0027, -0.0018,  0.0002],  
        [     0,  1.0000, -0.0006,  0.0001,  0.0207,  0.0008, -0.0000,  0.0005, -0.0003,  0.0006, -0.0038, -0.0003],
        [     0,       0,  0.9960,  0.0001, -0.0006,  0.0184,  0.0000, -0.0000, -0.0001, -0.0005,  0.0005, -0.0003],
    
        [     0,       0,  0.0133,  0.9981,  0.0096,  0.0007,  0.0003,  0.0025,  0.0030, -0.0019, -0.0001, -0.0071],
        [     0,       0,  0.0129, -0.0012,  0.9986,  0.0076,  0.0002,  0.0015, -0.0029, -0.0010,  0.0076,  0.0012],
        [     0,       0, -0.0396,  0.0006, -0.0017,  0.9834,  0.0000,  0.0002, -0.0000,  0.0012, -0.0019,  0.0011],
    
        [     0,       0, -0.0202,  0.0075, -0.0473, -0.0555,  0.9909, -0.0032, -0.0029, -0.0907,  0.1183, -0.7797],
        [     0,       0, -0.0995, -0.0098, -0.1662, -0.0309, -0.0068,  0.9669,  0.0014,  0.1366, -0.4267,  0.1508],
        [     0,       0,  0.2266, -0.0289,  0.0505,  0.1060,  0.0029,  0.0050,  0.9712,  0.6622, -0.1306,  0.0434],
        [     0,       0, -0.0167,  0.0133, -0.0011, -0.0023,  0.0000, -0.0003, -0.0121,  0.8542, -0.0141,  0.0248],
    
        [     0,       0, -0.0193, -0.0034,  0.0104, -0.0077, -0.0002,  0.0117,  0.0022,  0.0250,  0.8149, -0.0197],
        [     0,       0,  0.0049, -0.0061,  0.0085, -0.0058,  0.0044,  0.0001,  0.0015,  0.0176, -0.0114,  0.8621]])
    
    
    B = np.array([[-0.0001,  0.0001, -0.0004,  0.0004],
                  [ 0.0000, -0.0002,  0.0000, -0.0006],
                  [-0.0000, -0.0000,  0.0001, -0.0040],
                  [-0.0003,  0.0002, -0.0006,  0.0133],
                  [-0.0002,  0.0004,  0.0001,  0.0129],
                  [-0.0000, -0.0002,  0.0000, -0.0396],
                  [ 0.0091,  0.0029,  0.0034, -0.0202],
                  [ 0.0068,  0.0270, -0.0003, -0.0995],
                  [-0.0029, -0.0047,  0.0244,  0.2266],
                  [-0.0000,  0.0000,  0.0073, -0.0167],
                  [ 0.0002, -0.0073, -0.0024, -0.0193],
                  [-0.0044, -0.0003, -0.0013,  0.0049]])
elif model_select == 2:
    A = np.array([
    [1.0000 ,        0,   -0.0002,    0.0096,    0.0031,   -0.0052,    0.0007,   0.0037,    0.0034,   -0.0210,    0.0063,    0.0271], 
    [     0 ,   1.0000,   -0.0009,   -0.0045,    0.0179,    0.0019,    0.0017,   0.0041,   -0.0008,    0.0019,    0.0087,    0.0116], 
    [     0 ,        0,    0.9851,    0.0595,    0.0933,   -0.0012,   -0.0239,   0.0864,   -0.0118,   -0.0381,   -0.0764,   -0.0012], 
    [     0 ,        0,   -0.0010,    0.8633,   -0.1564,   -0.0008,    0.0901,  -0.1560,    0.0218,    0.0734,    0.1417,    0.0454], 
    [     0 ,        0,    0.0003,   -0.1302,    0.8409,    0.0105,    0.0907,  -0.1647,    0.0218,    0.0597,    0.0744,    0.2472], 
    [     0 ,        0,   -0.0363,    0.0254,    0.0600,    0.9726,   -0.0173,   0.0455,   -0.0021,   -0.0432,   -0.0116,   -0.0326], 
    [     0 ,        0,    0.0261,    0.0859,    0.1287,   -0.0004,    0.9266,   0.1462,   -0.0188,   -0.0369,   -0.0829,   -0.2062], 
    [     0 ,        0,    0.0394,   -0.0154,   -0.0937,    0.0399,   -0.0189,   0.9719,   -0.0023,    0.1016,   -0.2105,    0.1236], 
    [     0 ,        0,    0.0755,   -0.0109,    0.0722,    0.0586,    0.0279,   0.0083,    0.9713,    0.2726,   -0.0792,    0.0357], 
    [     0 ,        0,    0.0031,   -0.0248,   -0.0478,    0.0753,    0.0259,  -0.0087,   -0.0174,    0.9000,   -0.0333,   -0.0138], 
    [     0 ,        0,    0.0007,   -0.0066,    0.0386,   -0.0458,   -0.0041,   0.0822,   -0.0050,    0.0514,    0.8091,    0.0006], 
    [     0 ,        0,   -0.0015,   -0.1527,   -0.2472,    0.0071,    0.0999,  -0.1885,    0.0210,    0.0921,    0.1135,    0.9362]]) 
    
    B = np.array([ [-0.0007,   -0.0006,   -0.0027,   -0.0002],
                   [-0.0017,   -0.0009,    0.0005,   -0.0009],
                   [ 0.0239,    0.0081,   -0.0062,   -0.0149],
                   [-0.0901,   -0.0263,    0.0144,   -0.0010],
                   [-0.0907,   -0.0266,    0.0148,    0.0003],
                   [ 0.0173,    0.0067,   -0.0065,   -0.0363],
                   [ 0.0734,    0.0156,   -0.0121,    0.0261],
                   [ 0.0189,    0.0016,    0.0050,    0.0394],
                   [-0.0279,   -0.0071,    0.0190,    0.0755],
                   [-0.0259,   -0.0035,    0.0147,    0.0031],
                   [ 0.0041,   -0.0071,   -0.0102,    0.0007],
                   [-0.0999,   -0.0265,    0.0192,   -0.0015]])

DT_NOMINAL = 0.02 # [s]

class LinearModel(ModelBase):
    def __init__(self):
        self.x = np.zeros(13)
        ori_init = tft.quaternion_from_euler(radians(-160), 0, 0, 'rzyx') # xyzw
        self.x[5:9] = np.concatenate([[ori_init[3]], ori_init[:3]]) # wxyz
        #self.x[6] = 1.0 # identity quaternion
        self.u = np.zeros(4)
        
    def set_input(self, u, dt):
        self.u = u
        if not u[4]:
            self.u = np.concatenate([self.get_ypr_deg(), [0.0]])
        else:
            self.u = u[:4]

        
    def update(self, dt):
        dt_rel_err = (dt - DT_NOMINAL)/DT_NOMINAL
        if abs(dt_rel_err) > 0.1:
            rospy.logwarn('dt = %f (relative error %f)' % (dt, dt_rel_err))
        #u_ypra = self.u[:4]
        ypr = self.get_ypr_deg()
        x_ypr = np.concatenate([self.x[:6], ypr, self.x[10:13]])
        rospy.loginfo('x_ypr = ' + str(x_ypr))
        #rospy.loginfo(str(A.shape) + str(x_ypr.shape) + str(B.shape) + str(self.u.shape))
        rospy.loginfo('u = ' + str(self.u))
        x_ypr_next = np.dot(A, x_ypr) + np.dot(B, self.u)
        rospy.loginfo('x_ypr_next = ' + str(x_ypr_next))
        if x_ypr_next[2] == 0.0 and x_ypr_next[5] > 0:
            # On the ground..
            x_ypr_next[3:6] = np.zeros(3)
            x_ypr_next[6:9] = (ypr[0], 0, 0)

        ori_quat = tft.quaternion_from_euler(*(np.radians(x_ypr_next[6:9])), axes='rzyx') # xyzw
        self.x = np.concatenate([x_ypr_next[:6], 
                                 [ori_quat[3]], ori_quat[:3],
                                 x_ypr_next[9:12]])
        rospy.loginfo('x = ' + str(self.x))
        
    def get_position(self):
        return self.x[0:3]
            
    def get_velocity(self):
        return self.x[3:6]
            
    def get_orientation(self):
        """
        Return orientation quaternion in (x, y, z, w) convention
        """
        return tuple(self.x[7:10]) + (self.x[6],)
    
    def get_ypr_deg(self):
        return np.degrees(tft.euler_from_quaternion(self.get_orientation(), 'rzyx'))
    
    def get_angular_velocity(self):
        return self.x[10:13]
            

        