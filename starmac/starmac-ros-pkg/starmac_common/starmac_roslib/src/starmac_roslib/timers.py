# Based on Tim Field's code posted to ros-users. 
# See: http://ros-users.122217.n3.nabble.com/Timers-in-rospy-td2298885.html

# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, Willow Garage
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

import roslib; roslib.load_manifest('rospy')
import rospy, threading, time

class TimerEvent(object):
    def __init__(self, last_expected, last_real, current_expected, current_real, last_duration):
        self.last_expected    = last_expected
        self.last_real        = last_real
        self.current_expected = current_expected
        self.current_real     = current_real
        self.last_duration    = last_duration

class Timer(threading.Thread):
    def __init__(self, duration, callback, oneshot=False):
        threading.Thread.__init__(self)
        self._duration = duration
        self._callback = callback
        self._oneshot  = oneshot
        self.setDaemon(True)
        self.start()

    def run(self):
        r = rospy.Rate(1.0 / self._duration.to_sec())
        current_expected = rospy.Time.now() + self._duration
        last_expected, last_real, last_duration = None, None, rospy.Duration(0)
        while not rospy.is_shutdown():
            r.sleep()
            current_real = rospy.Time.now()
            #start = time.time()
            start = rospy.Time.now()
            self._callback(TimerEvent(last_expected, last_real, current_expected, current_real, last_duration))
            if self._oneshot:
                break
            #last_duration = time.time() - start
            last_duration = rospy.Time.now() - start
            last_expected, last_real = current_expected, current_real
            current_expected += self._duration

if __name__ == '__main__':
    import time
    rospy.init_node('test')
    def callback(event):
        print 'last_expected:        ', event.last_expected
        print 'last_real:            ', event.last_real
        print 'current_expected:     ', event.current_expected
        print 'current_real:         ', event.current_real
        print 'current_error:        ', (event.current_real - event.current_expected).to_sec()
        print 'profile.last_duration:', event.last_duration.to_sec()
        if event.last_real:
            print 'last_error:           ', (event.last_real - event.last_expected).to_sec(), 'secs'
        print

    Timer(rospy.Duration(0.05), callback)
    time.sleep(15)
