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

import roslib
roslib.load_manifest('starmac_tools')
import rosbag
import sys
import numpy as np
import scipy.io as sio

_primitive_types = [ 'byte', 'int8', 'uint8',
                    'int16', 'uint16', 'int32', 'uint32',
                    'int64', 'uint64', 'float32', 'float64', 'bool']

def mangle(s):
    """
    Function to mangle field names, replacing first underscores with double underscores,
    then slashes with underscores
    """
    mangled = s.replace('_','__').replace('/','_')
    assert(unmangle(mangled)) == s
    return mangled

def unmangle(s):
    return s[::-1].replace('__','~')[::-1].replace('_', '/').replace('~','_')

class BagLoader(object):
    """
    Class that allows one to load data from a ROS .bag file. 
    
    For the impatient:
    >>> mybag = BagLoader('some_bagfile.bag')
    >>> t = mybag._grey_innerloop_time
    >>> V = mybag._grey_innerloop_battVoltage
    >>> from pylab import *
    >>> plot(t, V)    
    
    Data is stored in the instance private member
    dictionary _data, with keys that are based on the ROS 'name' of each time-series variable (the ROS topic name
    as recorded in the bagfile is concatenated with slashes with the field name, i.e. /namespace/topic/field). 
    The variables are also available as instance members, with name mangling to convert slashes to underscores.
    (this is handy when you have some tab completion as in IPython)
    
    Note that at present, 'nested' fields, i.e. toplevel fields that are not a ROS 'primitive type' are not supported
    and are ignored, with the exception of fields of type Header in the top level, for which the stamp field is retrieved
    as a special case.
    
    Each topic includes the special field 'time' -- note that this is the time at which the message was recorded
    by rosbag. If the message includes a Header field, then at present the timestamp within cannot be accessed,
    as nested variables are not yet handled.
    
    Another useful feature is the ability to save the data to a Matlab-readable .mat file:
    >>> mybag.save_mat('test.mat')
    
    Note - the minimum recorded timestamp in the bagfile is taken to be time zero, and the message and header (when present)
    timestamps subtract this value (self.start_stamp) in 64-bit integer arithmetic before storing the result as a double. Thus
    the time values in the output are double (64-bit float) times in nanoseconds with zero being the aforementioned quantity.
    Note that this could lead to large negative times in cases where the timestamp is set to zero (may happen in headers). It is
    the user's responsibility to handle such a case appropriately. 
    """
    def __init__(self, filename, verbose=False, skip_images=True):
        self.verbose = verbose
        self._load(filename)
        self.filename = filename
        
    def _load(self, filename):
        # Load it up:
        print "Loading data from", filename
        self.b = rosbag.Bag(filename)
        # Figure out what time zero should be:
        self.start_stamp  = min([index[ 0].time for index in self.b._connection_indexes.values()])
        # Get the list of topics:
        topics = set([c.topic for c in self.b._get_connections()])                
        topic_datatypes    = {}
        topic_conn_counts  = {}
        topic_msg_counts   = {}
        topic_freqs_median = {}
        # And figure out a bunch of stuff about them:
        self.topic_msg_counts = {}
        for topic in topics:
            connections = list(self.b._get_connections(topic))
            stamps = [entry.time.to_sec() for entry in self.b._get_entries(connections)]
            
            topic_datatypes[topic] = connections[0].datatype
            topic_conn_counts[topic] = len(connections)
            self.topic_msg_counts[topic] = len(stamps)
        #    if len(stamps) > 1:
        #        periods = [s1 - s0 for s1, s0 in zip(stamps[1:], stamps[:-1])]
        #        med_period = _median(periods)
        #        if med_period > 0.0:
        #            topic_freqs_median[topic] = 1.0 / med_period
        
        self._data = {} # this is where everything will go
        self._ros_types = {}
        
        for topic in topics:
            print "Loading topic %s, type %s" % (topic, topic_datatypes[topic]),
            if topic_datatypes[topic] == 'sensor_msgs/Image':
                print ".. Skipping"
            else:
                print 
                self._load_process_topic(topic)
                
    def _load_process_topic(self, topic):
        msg_iter = self.b.read_messages(topic)
        i = 0
        N = self.topic_msg_counts[topic]
        #(topic, msg, time) = msg_iter.next()
        for (topic, msg, time) in msg_iter:
            if i == 0:
                #print 'Topic: ', topic, '(', msg._type, ')'
                #(fieldnames, types) = msg.__slots__, msg._get_types()
                self._data[topic+'/_time'] = np.zeros(shape=(N,), dtype=np.float64)
                self.__dict__[mangle(topic+'/_time')] = self._data[topic+'/_time']
            self._sub_load_init_array(i, 0, topic, msg, time, N)
            self._data[topic+'/_time'][i] = (time-self.start_stamp).to_sec()
                #fieldnames, types = self._load_init_arrays(topic, msg, time)
            #self._load_process_msg(i, topic, msg, time, fieldnames, types)
            i += 1

    
    def _sub_load_init_array(self, i, depth, base, msg, time, N):
        verbose = self.verbose
        (fieldnames, types) = msg.__slots__, msg._get_types()
        for fieldname2, type2 in zip(fieldnames, types):
            t_stripped = type2.split('[')[0]
            is_primitive = t_stripped in _primitive_types
            is_array = ('[' in type2 and ']' in type2)
            full_field_name = base + '/' + fieldname2
            array_size = None
            if type2 is 'time':
                for s in ('/secs', '/nsecs'):
                    if i == 0:
                        self._data[full_field_name + s] = np.zeros(shape=(N,), dtype=np.uint32) 
                        self._ros_types[full_field_name + s] = (t_stripped, is_primitive, is_array, array_size)
                        self.__dict__[mangle(full_field_name + s)] = self._data[full_field_name+s]
                        if verbose: print full_field_name + s, ':', str(self._data[full_field_name+s].dtype), str(self._data[full_field_name+s].shape)
                self._data[full_field_name + '/secs'][i] = msg.__getattribute__(fieldname2).secs
                self._data[full_field_name + '/nsecs'][i] = msg.__getattribute__(fieldname2).nsecs
            else:
                if is_primitive:
                    if is_array:
                        array_size_str = type2.split('[')[1].split(']')[0]
                        if len(array_size_str) > 0: # fixed length array
                            array_size = int(array_size_str)
                            if i == 0:
                                self._data[full_field_name] = np.zeros(shape=(N,array_size), dtype=t_stripped) 
                                if verbose: print " "*depth + full_field_name, ':', str(self._data[full_field_name].dtype), str(self._data[full_field_name].shape)
                            self._data[full_field_name][i,:] = msg.__getattribute__(fieldname2)
                        else:
                            if i == 0:
                                array_size = None # variable length array
                                self._data[full_field_name] = []
                                if verbose: print " "*depth + full_field_name, ':', type2
                            if t_stripped == 'uint8':
                                self._data[full_field_name].append(np.fromstring(msg.__getattribute__(fieldname2), dtype='uint8'))
                            else:
                                self._data[full_field_name].append(msg.__getattribute__(fieldname2))
                    else:
                        if i == 0:
                            self._data[full_field_name] = np.zeros(shape=(N,), dtype=t_stripped) 
                            if verbose: print " "*depth + full_field_name, ':', str(self._data[full_field_name].dtype), str(self._data[full_field_name].shape)
                        self._data[full_field_name][i] = msg.__getattribute__(fieldname2)
                    if i == 0:
                        self.__dict__[mangle(full_field_name)] = self._data[full_field_name]
                elif t_stripped == 'string':
                    if is_array:
                        if i == 0:
                            self._data[full_field_name] = []
                        self._data[full_field_name].append(msg.__getattribute__(fieldname2))
                    else:
                        if i == 0:
                            self._data[full_field_name] = []
                            if verbose: print " "*depth + full_field_name, ': string'
                        self._data[full_field_name].append(msg.__getattribute__(fieldname2))
                    if i == 0:
                        self.__dict__[mangle(full_field_name)] = self._data[full_field_name]
                else:
                    if is_array:
                        if i == 0:
                            print "WARNING: Arrays of non-primitive types not yet supported (%s : %s)" % (fieldname2, type2)
                    else:
                        if i == 0:
                            if verbose: print " "*depth + full_field_name, type2, "*"
                        self._sub_load_init_array(i, depth+1, full_field_name, msg.__getattribute__(fieldname2), time, N)
                            #print " "*depth + "<<< done recursing on ", full_field_name
                if i == 0:
                    self._ros_types[full_field_name] = (t_stripped, is_primitive, is_array, array_size)
                
    def save_mat(self, mat_filename):
        """
        Save data to a MATLAB (version 5) .mat file
        """
        mdict = {}
        for k, v in self._data.iteritems():
            if v is not None:
                mdict['v'+mangle(k)] = v # Matlab variable names must start with a letter
        sio.savemat(mat_filename, mdict, long_field_names=True)
            
if __name__ == "__main__":
    self = BagLoader(sys.argv[1])
    from pylab import *
    t = self.odom___time
    h = -self.odom_pose_pose_position_z
    plot(t,h)
    title('Altitude vs. Time')
    xlabel('Time [s]')
    ylabel('Altitude [m]')
    print "Testing save to .mat..."
    self.save_mat('test.mat')
    show()

