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

from math import sin, cos, radians, degrees
import numpy as np

import tf.transformations as tft

from starmac_roslib.pid import PidController

from model_base import ModelBase


GRAVITY = 9.81 # m/s/s
MASS = 1.5 # kg
THRUST_SCALING = MASS*GRAVITY/1500
LINEAR_DRAG = 0.1 # N/(m/s)
QUADRATIC_DRAG = 0.3 # N/(m/(s^2))
EPS = 0.0001

def clip(x, lb, ub):
    if x > ub:
        return ub, True
    elif x < lb:
        return lb, True
    else:
        return x, False

class SimpleNonlinearModel(ModelBase):
    """
    Simple nonlinear model of arbitrary quadrotor type vehicle.
    13 states:
    index    quantity                    units
    0        x position                    m
    1        y position                    m
    2        z position                    m
    
    3        x velocity                    m/s
    4        y velocity                    m/s
    5        z velocity                    m/s
    
    6        orientation quaternion (w)   
    7        orientation quaternion (x)   
    8        orientation quaternion (y)   
    9        orientation quaternion (z)
       
    10       angular velocity (x)          rad/s
    11       angular velocity (y)          rad/s
    12       angular velocity (z)          rad/s
    
    4 inputs:
    index    quantity  units
    0        yaw       deg
    1        pitch     deg
    2        roll      deg
    3        altitude  m
    4        enable    (bool, True if motors are on)
    
    NOTE: transformations.py (tf.transformations) represents quaternions 
          as (x, y, z, w) tuples!
    """
    def __init__(self):
        self.x = np.zeros(13)
        self.x[6] = 1.0 # identity quaternion
        self.u = np.zeros(5)
        # Grab parameters:
        self.alt_KP = rospy.get_param("~alt_KP", 600.0)
        self.alt_KI = rospy.get_param("~alt_KI", 400.0)
        self.alt_KD = rospy.get_param("~alt_KD", 600.0)
        #self.alt_KDD = rospy.get_param("~alt_KDD",50.0)
        self.alt_Ilimit = rospy.get_param("~alt_Ilimit", 400.0)
        #self.yaw_KP = rospy.get_param("~yaw_KP", 4.0)
        #self.yaw_KI = rospy.get_param("~yaw_KI", 0.0)
        #self.yaw_KD = rospy.get_param("~yaw_KD", 0.0)
        #self.yaw_Ilimit = rospy.get_param("~yaw_Ilimit", 50.0)
        self.nominal_thrust = rospy.get_param("~nominal_thrust", 1500)
        self.thrust_mult = rospy.get_param("~thrust_mult", 1.0)
        self.max_thrust = rospy.get_param("~max_thrust", 2100)
        
        self.prev_orientation = (0.0, 0.0, 0.0, 1.0)

        self.alt_pid = PidController(self.alt_KP, self.alt_KI, self.alt_KD, self.alt_Ilimit)
        
    def set_input(self, u, dt):
        self.u = u
        self.thrust_cmd = self.nominal_thrust + \
                  self.alt_pid.update(-self.get_position()[2] - u[3], dt)
        
    def update(self, dt):
        # The following model is completely arbitrary and should not be taken to be representative of
        # real vehicle performance!
        # But, it should be good enough to test out control modes etc.
        yaw_cmd, pitch_cmd, roll_cmd, alt_cmd, motor_enable = self.u
        cur_yaw, cur_pitch, cur_roll = tft.euler_from_quaternion(self.get_orientation(), 'rzyx')
        
        # Feedback control: attitude angles track inputs exactly; altitude is input to
        # PID controller for thrust
        if motor_enable:
            thrust = self.thrust_cmd
            # accelerations due to pitch/roll in body frame:
            pitch_clipped, pwas_clipped = clip(pitch_cmd, -50, 50)
            roll_clipped, rwas_clipped = clip(roll_cmd, -50, 50)
            yaw_cmd = yaw_cmd
        else:
            thrust = 0.0
            yaw_cmd = degrees(cur_yaw)
            pitch_clipped = degrees(cur_pitch)
            roll_clipped = degrees(cur_roll)
            pwas_clipped = rwas_clipped = False
            
        # Propagate dynamics
        # Velocity
        accel_thrust = thrust*THRUST_SCALING/MASS
        if pwas_clipped or rwas_clipped:
            rospy.logwarn('Pitch and/or roll clipped! Pre-clip values %f %f' % (pitch_cmd, roll_cmd))
        accel_body = np.array([-sin(radians(pitch_clipped))*accel_thrust, 
                              sin(radians(roll_clipped))*accel_thrust, 
                              0.0]) # vertical accel will be added in inertial frame
        # rotate body frame accelerations into inertial frame, and add drag and vertical forces:
        Ryaw = tft.euler_matrix(cur_yaw, 0, 0, 'rzyx')
        accel_inertial_lateral = np.dot(Ryaw[:3,:3], accel_body)
        cur_velocity = self.get_velocity()
        accel_inertial = accel_inertial_lateral + \
                            np.array([- LINEAR_DRAG*cur_velocity[0]
                                      - QUADRATIC_DRAG*np.sign(cur_velocity[0])*(cur_velocity[0]**2),
                        
                                      - LINEAR_DRAG*cur_velocity[1]
                                      - QUADRATIC_DRAG*np.sign(cur_velocity[1])*(cur_velocity[1]**2),
                       
                                      GRAVITY - accel_thrust*cos(radians(pitch_clipped))*cos(radians(roll_clipped))])
        velocity = cur_velocity + accel_inertial*dt
        # Position
        position = self.get_position() + velocity*dt
        position[2] = min(0, position[2]) # can't go lower than the ground
        
        # Angles
        if position[2] == 0.0 and velocity[2] > 0:
            # On the ground..
            velocity = np.zeros(3)
            orientation = tft.quaternion_from_euler(radians(yaw_cmd), 0, 0, 'rzyx')
        else:
            # assume perfect yaw rate, and pitch and roll tracking:
            orientation = tft.quaternion_from_euler(radians(yaw_cmd), radians(pitch_clipped), radians(roll_clipped), 'rzyx')
            
        # Angular rate
        # from J. Diebel, "Representing attitude: Euler angles, unit quaternions, and rotation vectors," 2006
        q_cur = np.array((orientation[3],) + tuple(orientation[:3])) # convert to wxyz 
        q_prev = np.array((self.prev_orientation[3],) + tuple(self.prev_orientation[:3])) # convert to wxyz
        if dt > EPS:
            q_dot = (q_cur - q_prev)/dt
        else:
            q_dot = np.zeros(4)  
        Wprime = np.array([[-q_cur[1],  q_cur[0],  q_cur[3], -q_cur[2]],
                           [-q_cur[2], -q_cur[3],  q_cur[0],  q_cur[1]],
                           [-q_cur[3],  q_cur[2], -q_cur[1],  q_cur[0]]])
        ang_vel_body = 2*np.dot(Wprime,q_dot)
        
        # Assemble new state vector
        self.x = np.concatenate([position, velocity, q_cur, ang_vel_body])
        
        # Save for future finite differences
        self.prev_orientation = orientation[:] # be sure to copy not reference...
            
    def get_position(self):
        return self.x[0:3]
            
    def get_velocity(self):
        return self.x[3:6]
            
    def get_orientation(self):
        """
        Return orientation quaternion in (x, y, z, w) convention
        """
        return tuple(self.x[7:10]) + (self.x[6],)
    
    def get_angular_velocity(self):
        return self.x[10:13]
            

        