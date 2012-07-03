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

import sys
from load_bag import BagLoader
from timeseries import uniform_resample, quaternion_to_eulerypr
import scipy.io as sio
import numpy as np

in_bag, out_mat = sys.argv[1:3]

input_bag = BagLoader(in_bag)

quats = (input_bag.odom_pose_pose_orientation_w, 
         input_bag.odom_pose_pose_orientation_x, 
         input_bag.odom_pose_pose_orientation_y,
         input_bag.odom_pose_pose_orientation_z)
yaw, pitch, roll = [np.degrees(x) for x in quaternion_to_eulerypr(quats)]

resample_inputs = (('linear',
                    input_bag.odom___time,
                    input_bag.odom_pose_pose_position_x,
                    input_bag.odom_pose_pose_position_y,
                    input_bag.odom_pose_pose_position_z,
                    input_bag.odom_twist_twist_linear_x,
                    input_bag.odom_twist_twist_linear_y,
                    input_bag.odom_twist_twist_linear_z,
                   ),
                   ('zero',
                    input_bag.odom___time,
                    input_bag.odom_pose_pose_orientation_w,
                    input_bag.odom_pose_pose_orientation_x,
                    input_bag.odom_pose_pose_orientation_y,
                    input_bag.odom_pose_pose_orientation_z,
                    ),
                   ('zero',
                    input_bag.odom___time,
                    yaw,
                    pitch,
                    roll,
                    ),
                   ('linear',
                    input_bag.asctec__proc_imu___time,
                    input_bag.asctec__proc_imu_angular__velocity_x,
                    input_bag.asctec__proc_imu_angular__velocity_y,
                    input_bag.asctec__proc_imu_angular__velocity_z
                    ),
                    ('zero',
                     input_bag.controller__mux_output___time,
                     input_bag.controller__mux_output_yaw__cmd,
                     input_bag.controller__mux_output_pitch__cmd,
                     input_bag.controller__mux_output_roll__cmd,
                     input_bag.controller__mux_output_alt__cmd
                     )
                  )

t_out, resample_outputs = uniform_resample(resample_inputs, 0.02) # resample at 50 Hz

mdict = {'t': t_out,
         
         # States:
         'x': resample_outputs[0][0],
         'y': resample_outputs[0][1],
         'z': resample_outputs[0][2],
         'vx': resample_outputs[0][3],
         'vy': resample_outputs[0][4],
         'vz': resample_outputs[0][5],
         
         'qw': resample_outputs[1][0],
         'qx': resample_outputs[1][1],
         'qy': resample_outputs[1][2],
         'qz': resample_outputs[1][3],
         
         'yaw':   resample_outputs[2][0],
         'pitch': resample_outputs[2][1],
         'roll':  resample_outputs[2][2],
         
         'wx': resample_outputs[3][0],
         'wy': resample_outputs[3][1],
         'wz': resample_outputs[3][2],
         
         # Inputs:
         'u_yaw':   resample_outputs[4][0],
         'u_pitch': resample_outputs[4][1],
         'u_roll':  resample_outputs[4][2],
         'u_alt':   resample_outputs[4][3],
         }

print "Saving %s..." % out_mat 
sio.savemat(out_mat, mdict, long_field_names=True)
