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

import numpy as np

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
y = height
d1 = 0.4
d2 = 0.4

theta1 = 0.6754
theta2 = 1.7802

# put the robot in a stable positiion		
def initialize():
	i = 0
	while i < 10:
		ref.ref[ha.RSR] -= 0.01
		ref.ref[ha.LSR] += 0.01
		i += 0.5
		r.put(ref)
		time.sleep(.01)
	print("ROBOT - initialize() done")

#crouch the robot		
def init_ready():
	i = 0
	while i < 0.7:
		ref.ref[ha.RAP] = -i
		ref.ref[ha.LAP] = -i
		ref.ref[ha.LKN] = 2*i
		ref.ref[ha.RKN] = 2*i
		ref.ref[ha.RHP] = -i
		ref.ref[ha.LHP] = -i
		r.put(ref)
		i += 0.05
		time.sleep(0.5)
	print ("ROBOT - init_ready() done")

# put the robot in stand position
def stand():
	ref.ref[ha.RAP] = 0
	ref.ref[ha.LAP] = 0
	ref.ref[ha.LKN] = 0
	ref.ref[ha.RKN] = 0
	ref.ref[ha.RHP] = 0
	ref.ref[ha.LHP] = 0
	r.put(ref)
	time.sleep(0.5)
	print("ROBOT - stand() done")

#initialize()
time.sleep(1)
init_ready()
time.sleep(10)
stand()
time.sleep(1)
print("DONE")



