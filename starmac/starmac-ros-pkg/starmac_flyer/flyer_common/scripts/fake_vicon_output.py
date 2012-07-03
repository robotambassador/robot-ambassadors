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

import tf.transformations as tft
import numpy as np
r = Rate(50)
dummy_msg = msg.geometry_msgs.TransformStamped()
dummy_msg.transform.rotation.w = 1.0
dummy_msg.header.frame_id = '/enu'
dummy_msg.child_frame_id = '/pelican1/flyer_vicon'
tfl=transforms._config.listener
while (True):
    dummy_msg.header.stamp = now()
    try:
        t_imu_vicon, q_imu_vicon = tfl.lookupTransform('/pelican1/flyer_imu','/pelican1/flyer_vicon',now())
        T_imu_vicon = np.dot(tft.translation_matrix(t_imu_vicon),tft.quaternion_matrix(q_imu_vicon)) 
    except:
        loginfo('exception trying to get vicon-imu')
    try:
        latest_autosequence_info = topics.pelican1.control_mode_autosequence.info[0]
        if latest_autosequence_info is not None:
            # oh this is so annoying..
            T_ned_imu_norot = tft.translation_matrix([latest_autosequence_info.north_cmd,
                                                latest_autosequence_info.east_cmd,0])
            T_ned_imu = np.dot(T_ned_imu_norot,tft.quaternion_matrix(tft.quaternion_from_euler(np.radians(latest_autosequence_info.yaw_cmd), 0, 0, 'rzyx')))
            T_enu_ned = tft.euler_matrix(np.radians(180),0,np.radians(-90),'rxyz')
            T_enu_imu = np.dot(T_enu_ned, T_ned_imu)
            T_enu_vicon = np.dot(T_enu_imu,T_imu_vicon)
            q = tft.quaternion_from_matrix(T_enu_vicon)
            dummy_msg.transform.translation.x = T_enu_vicon[0,3]
            dummy_msg.transform.translation.y = T_enu_vicon[1,3]
            #q = tft.quaternion_from_euler(-np.radians(90.0), 0, 0, 'rzyx')
            dummy_msg.transform.rotation.x = q[0]
            dummy_msg.transform.rotation.y = q[1]
            dummy_msg.transform.rotation.z = q[2]
            dummy_msg.transform.rotation.w = q[3]
    except:
        loginfo('exception 2')
    #loginfo('Publishing dummy vicon message')
    topics['/pelican1/vicon_recv_direct/output'](dummy_msg)
    r.sleep()