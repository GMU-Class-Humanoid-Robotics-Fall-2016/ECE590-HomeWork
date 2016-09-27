#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */

#Code modified from hubo-simple-demo-python by Dr.Laforo

import hubo_ach as ha
import ach
import sys
import time
from ctypes import *
import math

if len(sys.argv) == 2:
	print 'Using given height'
	height = float(sys.argv[1])
else:
	print 'Usage crouch.py x :  x is the height from ground. Using default :0.5'
	height = 0.5

	
# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

# x is 0 
y = height #0.5 for assignment
d1 = 0.4
d2 = 0.4 
# d1 + d2 = 0.8 which is height of robots waist from 

def calc_theta_2(y,d1,d2):
	numerator = y**2 - d1**2 - d2**2
	denominator = 2 * d1 * d2
	theta2 = math.acos(float(numerator)/denominator)
	return theta2

def calc_theta_1(y, d1, d2, theta2):
	numerator = y * (d1 + d2*math.cos(theta2))
	denominator = y*(d2*math.sin(theta2))
	theta1 = math.atan2(numerator,denominator)
	return theta1

theta2 = calc_theta_2(y,d1,d2)
theta1 = calc_theta_1(y,d1,d2,theta2)
theta2b = math.pi - theta2
factor = theta2b/theta1

print "Theta_1 is %f Theta_2 is %f" %(theta1,theta2b)

#crouch the robot		
def crouch():
	i = 0;
	while i < theta1:
		ref.ref[ha.RAP] = -i
		ref.ref[ha.LAP] = -i
		ref.ref[ha.LKN] = factor*i
		ref.ref[ha.RKN] = factor*i
		ref.ref[ha.RHP] = -i
		ref.ref[ha.LHP] = -i
		r.put(ref)
		i += 0.05
		time.sleep(0.5)
	print ("ROBOT - crouch() done")

# put the robot in stand position
def stand():
	i = theta1
	while i > 0:
		ref.ref[ha.RAP] = -i
		ref.ref[ha.LAP] = -i
		ref.ref[ha.LKN] = factor*i
		ref.ref[ha.RKN] = factor*i
		ref.ref[ha.RHP] = -i
		ref.ref[ha.LHP] = -i
		i -= 0.05
		r.put(ref)
		time.sleep(0.5)
	print("ROBOT - stand() done")
	
num = 0
while num < 5:
	crouch()
	time.sleep(10)
	stand()
	time.sleep(2)
	num += 1
print("DONE")


