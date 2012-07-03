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

from numpy import zeros, eye, dot, array
from numpy.linalg import inv, norm
from numpy.random import multivariate_normal
class DiscreteKalmanFilter:
    quiet=True
    def __init__(self, n, P_0_0, xhat_0_0=None, A=None, B=None, C=None, R=None, Q=None, quiet=True):
        self.quiet = quiet
        self.P_k_k = P_0_0
        self.P_k_km1 = self.P_k_k
        self.n = n
        if xhat_0_0 is None:
            self.xhat_k_k = zeros(n)
        else:
            self.xhat_k_k = xhat_0_0
        self.xhat_k_km1 = self.xhat_k_k
        if A is None:
            self.A = eye(n)
        else:
            self.A = A
        if B is None:
            self.no_input = True
            self.ni = 0
        else:
            self.no_input = False
            self.B = B
            self.ni = B.shape[1]
        if C is None:
            self.C = eye(n)
        else:
            self.C = C
        self.no = self.C.shape[0]
        if R is None:
            self.R = zeros((self.no,self.no))
        else:
            self.R = R
        if Q is None:
            self.Q = zeros((self.n,self.n))
        else:
            self.Q = Q
        if not self.quiet:
            self.print_info()
        
    def print_info(self):
        print "Kalman filter initialized"
        print "n = %d, ni = %d, no = %d" % (self.n, self.ni, self.no)
        print "A = "
        print self.A
        if self.no_input:
            print "no input"
        else:
            print "B = "
            print self.B
        print "C = "
        print self.C
        print "Q = "
        print self.Q
        print "R = "
        print self.R
        self.print_a_priori()
        self.print_a_posteriori()
        
    def print_a_posteriori(self):
        print "xhat_k_k = "
        print self.xhat_k_k
        print "P_k_k = "
        print self.P_k_k
    
    def print_a_priori(self):
        print "xhat_k_km1 = "
        print self.xhat_k_km1
        print "P_k_km1 = "
        print self.P_k_km1

        
    def predict(self, u=None):
        xhat_k_km1 = dot(self.A, self.xhat_k_k)
        if not self.no_input:
            xhat_k_km1 += dot(self.B, u)
        self.xhat_k_km1 = xhat_k_km1
        self.P_k_km1 = dot(dot(self.A,self.P_k_k),self.A.T) + self.Q
        
    def correct(self, y_k):
        # Calculate the innovation:
        innov = y_k - dot(self.C, self.xhat_k_km1)
        #print "innovation = ", innov
        # Innovation covariance:
        S_k = dot(self.C,dot(self.P_k_km1,self.C.T)) + self.R
        #print "S_k = "
        #print S_k
        # Optimal Kalman gain:
        K_k = dot(self.P_k_km1,dot(self.C.T,inv(S_k)))
        #print "K_k = "
        #print K_k
        # Updated (a posteriori) state estimate:
        self.xhat_k_k = self.xhat_k_km1 + dot(K_k,innov)
        # Updated (a posteriori) estimate covariance:
        self.P_k_k = dot(eye(self.n) - dot(K_k,self.C), self.P_k_km1)
        
if __name__ == "__main__":
    self = DiscreteKalmanFilter(3, eye(3)*100.0, zeros(3), R=eye(3)*10.0, quiet=False)
    true_value = array([1,0,0])
    while(True):
        self.predict()
        #self.print_a_priori()
        y_k = true_value + multivariate_normal(zeros(3), self.R)
        self.correct(y_k)
        print y_k, self.xhat_k_k, norm(self.xhat_k_k - true_value)
        #self.print_a_posteriori()
    
    