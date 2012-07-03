
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
misc.py

Miscellaneous routines
"""
from numpy import sign, fmod

def bound(x, limit):
    assert limit >= 0
    if x > limit:
        return limit
    elif x < -limit:
        return -limit
    else:
        return x
    
def clip(x, low_limit, high_limit):
    assert high_limit >= low_limit
    if x < low_limit:
        return low_limit
    elif x > high_limit:
        return high_limit
    else:
        return x
    
def mod180deg(x):
    if x >= 0:
        retval = fmod(x+180.0, 360.0)-180.0
    else:
        retval = -(fmod(-x+180.0, 360.0)-180.0)
    assert -180.0 <= retval <= 180.0
    return retval
    
def deadband(x, deadband):
    if -deadband < x < deadband:
        return 0.0
    else:
        return x-sign(x)*deadband
    
if __name__ == "__main__":
    assert bound(10, 8) == 8
    assert bound(0.5, 10) == 0.5
    assert deadband(10, 2.0) == 8.0
    assert deadband(1.999, 2.0) == 0.0
    assert deadband(-10, 2.0) == -8.0
    assert deadband(-0.5, 2.0) == 0.0
    assert mod180deg(45.0) == 45.0
    assert mod180deg(179.0) == 179.0
    assert mod180deg(-179.0) == -179.0
    assert mod180deg(182.0) == -178.0
    assert mod180deg(-182.0) == 178.0
    print "all tests passed"