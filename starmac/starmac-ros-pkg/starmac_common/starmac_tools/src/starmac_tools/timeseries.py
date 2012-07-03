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
timeseries.py

Miscellaneous functions for dealing with time series data

"""
import roslib
roslib.load_manifest('starmac_tools')
from scipy.interpolate import interp1d
import numpy as np
import tf.transformations as tft

def uniform_resample(inputs, dt):
  """
  In general time series data t, x will have non-uniform sampling interval and further
  in a large system there may be several sources of data with different sampling times.
  
  This function takes a tuple of tuples as input. Each of the inner tuples must contain as its
  first element a 'kind' argument (per the kind argument of scipy.interpolate.interp1d; you
  probably want to use either 'linear' or 'zero'). The second element of the tuple is a numpy array
  of input sample times, and as its remaining elements, 1-D arrays of the
  data to be resampled.
  
  The second argument, dt, is the desired output sample period.
  
  The output will first a vector of the new sample times, and second, a tuple of tuples, 
  the outer tuple of which is the same length as the input one.
  Each inner tuple will have the data, resampled as requested. 
  """
  # First find min and max time out of the whole data set
  input_times = [input[1] for input in inputs]
  t_min = np.min(np.concatenate(input_times))
  t_max = np.max(np.concatenate(input_times))
  print "t_min = ", t_min, " t_max = ", t_max
  outputs = []
  t_out = np.arange(t_min, t_max+dt/10, dt)
  for input in inputs:
    kind, t_in = input[:2]
    out_series = []
    for series in input[2:]:
      f = interp1d(t_in, series, kind, bounds_error=False, fill_value=0.0)
      out_series.append(f(t_out))
    outputs.append(out_series)
  return t_out, outputs

def quaternion_to_eulerypr(quat_series):
  """
  Convert a series of quaternions to the equivalent Euler angle (yaw-pitch-roll) representation.
  
  Input: tuple of 1-d vectors (w, x, y, z)
  Output: tuple of 1-d vectors (yaw, pitch, roll) in radians
  """
  # This is surely not the most efficient way to do this...
  yaw = np.empty_like(quat_series[0])
  pitch = np.empty_like(quat_series[0])
  roll = np.empty_like(quat_series[0])
  for i in range(len(quat_series[0])):
    yaw[i], pitch[i], roll[i] = \
      tft.euler_from_quaternion((quat_series[1][i], quat_series[2][i], quat_series[3][i], quat_series[0][i]), 
                                 axes='rzyx')
  return yaw, pitch, roll
  
if __name__ == "__main__":
  t1 = np.linspace(0, 10, 10)
  x1 = np.sin(t1)
  t2 = np.linspace(0.1,12.5,20)
  x2 = np.cos(t2)
  tout, data_out = uniform_resample((('linear', t1, x1), ('zero', t2, x2)), 0.01)
  #
  N=10
  quats = (np.empty(N), np.empty(N), np.empty(N), np.empty(N))
  for i in range(len(quats[0])):
    quats[1][i], quats[2][i], quats[3][i], quats[0][i] = \
      tft.quaternion_from_euler(np.radians(180*i/N), np.radians(-90*i/N), np.radians(45*i/N), 'rzyx')
  yaw, pitch, roll = quaternion_to_eulerypr(quats)
  