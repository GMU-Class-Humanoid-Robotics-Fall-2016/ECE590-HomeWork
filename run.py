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

step_distance = 0.05
step_count = 0
step_end = 3/step_distance
length_1 = 0.34003
length_2 = 0.34038
length_3 = 0.11497

RAP_final = 0
RKN_final = 0
RHP_final = 0
LAP_final = 0
LKN_final = 0
LHP_final = 0
RAR_final = 0
LHR_final = 0
LAR_final = 0
RHR_final = 0


if len(sys.argv) == 2:
	print 'Using given step distance'
	step_distance = float(sys.argv[1])
else:
	print 'Using default step distance: 0.05'
	
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

def three_dof(theta_1,theta_2,theta_3):
	length = length_1*np.cos(theta_1)+length_2*np.cos(theta_2 + theta_3)+length_3
	return length
		
# put the robot in a stable positiion		
def initialize():
	i = 0
	while i < 10:
		ref.ref[ha.RSR] -= 0.01
		ref.ref[ha.LSR] += 0.01
		i += 0.5
		r.put(ref)
		time.sleep(.01)
	print("Robot initialization complete")

#crouch the robot		
def init_ready():
	i = 0
	while i < .5:
		ref.ref[ha.RAP] = -i
		ref.ref[ha.LAP] = -i
		ref.ref[ha.LKN] = 2*i
		ref.ref[ha.RKN] = 2*i
		ref.ref[ha.RHP] = -i
		ref.ref[ha.LHP] = -i
		r.put(ref)
		i += 0.05
		time.sleep(0.5)
	print("Robot ready")
		
# lean the robot forward
def init_lean():
	i=0
	while i < .15:
		ref.ref[ha.RAR] = i
		ref.ref[ha.LAR] = i
		ref.ref[ha.RHR] = -i
		ref.ref[ha.LHR] = -i
		r.put(ref)
		i += 0.01
		time.sleep(.5)
	print("Robot lean")
	
def init_done():
	i = 0.5
	while i <0.75:
		ref.ref[ha.RAP] = -i
		ref.ref[ha.RKN] = 2*i
		ref.ref[ha.RHP] = -i
		r.put(ref)
		i += 0.05
		time.sleep(.5)
	global RAP_final
	global RKN_final
	global RHP_final
	global LAP_final
	global LKN_final
	global LHP_final
	global RAR_final
	global LAR_final
	global RHR_final
	global LHR_final


	RAP_final = ref.ref[ha.RAP]
	RKN_final = ref.ref[ha.RKN]
	RHP_final = ref.ref[ha.RHP]
	LAP_final = ref.ref[ha.LAP]
	LKN_final = ref.ref[ha.LKN]
	LHP_final = ref.ref[ha.LHP]
	RAR_final = ref.ref[ha.RAR]
	LAR_final = ref.ref[ha.LAR]
	RHR_final = ref.ref[ha.RHR]
	LHR_final = ref.ref[ha.LHR]
	print("Robot - done")

def right_step():
	dof_x_position = three_dof(ref.ref[ha.RHP], ref.ref[ha.RHP],ref.ref[ha.RKN] )
	dof_theta = np.arcsin(step_distance/dof_x_position)

	i = 0
	#raise right leg up
	while i < dof_theta:
		ref.ref[ha.RHP] -= 0.005
		ref.ref[ha.RAP] += 0.005
		i += 0.005
		r.put(ref)
		time.sleep(.05)

	right_leg = three_dof(ref.ref[ha.LHP],ref.ref[ha.LHP], ref.ref[ha.LKN])
	left_leg = three_dof(ref.ref[ha.RHP],ref.ref[ha.RHP],ref.ref[ha.RKN])

	#Right leg is higher than left leg
	#Adjust right leg slowly downwards
	while right_leg > left_leg:
		ref.ref[ha.LAP] -= 0.00005
		ref.ref[ha.LKN] += 0.0001
		ref.ref[ha.LHP] -= 0.00005
		r.put(ref)
		time.sleep(.00005)
		right_leg = three_dof(ref.ref[ha.LHP],ref.ref[ha.LHP], ref.ref[ha.LKN])
		left_leg = three_dof(ref.ref[ha.RHP],ref.ref[ha.RHP],ref.ref[ha.RKN])
		time.sleep(.0005)
	print("Robot Right step taken")
	
def left_step():
	dof_x_position = three_dof(ref.ref[ha.LHP], ref.ref[ha.LHP],ref.ref[ha.LKN])
	dof_theta = np.arcsin(step_distance/dof_x_position)

	i = 0
	#raise left leg up
	while i<dof_theta:
		ref.ref[ha.LHP] = ref.ref[ha.LHP]-.005
		ref.ref[ha.LAP] = ref.ref[ha.LAP]+.005
		i = i + .005
		r.put(ref)
		time.sleep(.05)

	left_leg = three_dof(ref.ref[ha.RHP],ref.ref[ha.RHP],ref.ref[ha.RKN])
	right_leg = three_dof(ref.ref[ha.LHP],ref.ref[ha.LHP], ref.ref[ha.LKN])	

	#Adjust left leg slowly downwards
	while left_leg > right_leg:
		ref.ref[ha.LAP] -= 0.00005
		ref.ref[ha.LKN] += 0.0001
		ref.ref[ha.LHP] -= 0.00005
		r.put(ref)
		time.sleep(.00005)
		left_leg = three_dof(ref.ref[ha.RHP],ref.ref[ha.RHP],ref.ref[ha.RKN])
		right_leg = three_dof(ref.ref[ha.LHP],ref.ref[ha.LHP], ref.ref[ha.LKN])
		time.sleep(.0005)
	print("Robot left step taken")
	
#shift the center of mass over the right
def shift_right_weight():
	p = three_dof(ref.ref[ha.LHP],ref.ref[ha.LHP], ref.ref[ha.LKN])
	l = np.sqrt((p)*(p) + (step_distance)*(step_distance))
	theta_r = ref.ref[ha.RAR]
	theta_hp = ref.ref[ha.LHP]
	theta_ap = ref.ref[ha.LAP]
	theta_n = 0.05*(l-p)
	rate_hp = np.abs(0.05*(ref.ref[ha.LHP]-ref.ref[ha.RHP]))
	rate_r = np.abs(0.1*theta_r)

	while theta_r >-ref.ref[ha.RAR] or ref.ref[ha.RHP] < theta_hp or ref.ref[ha.RAP] < theta_ap or p<l:
		if ref.ref[ha.RAP] > theta_ap:
			ref.ref[ha.LAP] = -(ref.ref[ha.LHP]+ref.ref[ha.LKN])
			ref.ref[ha.RAP] -= rate_hp
		if ref.ref[ha.RHP] < theta_hp:
			ref.ref[ha.LHP] += rate_hp
			ref.ref[ha.RHP] += rate_hp
		if p<l:
			ref.ref[ha.LKN] -= rate_hp
			p = three_dof(ref.ref[ha.LHP],ref.ref[ha.LHP], ref.ref[ha.LKN])
		if theta_r >-ref.ref[ha.RAR] :
			ref.ref[ha.RAR] -= rate_r
			ref.ref[ha.LAR] -= rate_r
			ref.ref[ha.RHR] += rate_r
			ref.ref[ha.LHR] += rate_r
		time.sleep(0.0001)
		r.put(ref)
		time.sleep(0.5)

	while ref.ref[ha.LHP] > -0.75:
		ref.ref[ha.LHP] -= 0.01
		ref.ref[ha.LKN] += 0.01
		r.put(ref)
		time.sleep(.1)
	print("Robot weight shifted over right")

#shift the center of mass over the left
def shift_left_weight():
	lp = three_dof(ref.ref[ha.RHP],ref.ref[ha.RHP],ref.ref[ha.RKN])
	l = np.sqrt((p)*(p) + (step_distance)*(step_distance))
	theta_l = ref.ref[ha.LAR]
	theta_hp = ref.ref[ha.RHP]
	theta_ap = ref.ref[ha.RAP]
	theta_n = 0.05*(l-p)
	rate_hp = np.abs(0.05*(ref.ref[ha.RHP]-ref.ref[ha.LHP]))
	rate_r = np.abs(0.1*theta_r)

	while theta_r <-ref.ref[ha.LAR] or ref.ref[ha.LHP] < theta_hp or ref.ref[ha.LAP] < theta_ap or p<l:
		if ref.ref[ha.LAP] > theta_ap:
			ref.ref[ha.RAP] = -(ref.ref[ha.RHP]+ref.ref[ha.RKN])
			ref.ref[ha.LAP] -= rate_hp
		if ref.ref[ha.LHP] < theta_hp:
			ref.ref[ha.RHP] += rate_hp
			ref.ref[ha.LHP] += rate_hp
		if p<l:
			ref.ref[ha.RKN] -= rate_hp
			p = three_dof(ref.ref[ha.RHP], ref.ref[ha.RHP], ref.ref[ha.RKN])
		if theta_r <-ref.ref[ha.LAR] :
			ref.ref[ha.LAR] += rate_r
			ref.ref[ha.RAR] += rate_r
			ref.ref[ha.LHR] -= rate_r
			ref.ref[ha.RHR] -= rate_r
		time.sleep(0.0001)
		r.put(ref)
		time.sleep(0.5)

	while ref.ref[ha.RHP] > -.75:
		ref.ref[ha.RHP] -= 0.01
		ref.ref[ha.RAP] -= 0.01
		ref.ref[ha.RKN] += 0.01
		r.put(ref)
		time.sleep(.1)
	print("Robot weight shifted over left")

# fix the robot to initial ready position	
def get_ready_right():
	time.sleep(5)
	check_one = 0
	check_two = 0
	check_three = 0
	check_four = 0
	check_five = 0
	check_six = 0
	while check_four+check_three+check_five+check_six+check_one+check_two!= 6:
		if ref.ref[ha.RKN] < LKN_final -0.001:
			ref.ref[ha.RKN] += 0.0005
		elif ref.ref[ha.RKN] > LKN_final +0.001: 	  
			ref.ref[ha.RKN] -= 0.0005
		else:
			check_four = 1 		
		if ref.ref[ha.LKN] < RKN_final -0.001:
			ref.ref[ha.LKN] += 0.0005
		elif ref.ref[ha.LKN] > RKN_final +.001: 	  
			ref.ref[ha.LKN] -= 0.0005
		else:
			check_three = 1
		if ref.ref[ha.LHP] < RHP_final -.001:
			ref.ref[ha.LHP] += 0.0005
		elif ref.ref[ha.LHP] > RHP_final +.001: 	  
			ref.ref[ha.LHP] -= 0.0005
		else:
			check_six = 1 
		if ref.ref[ha.RHP] < LHP_final -.001:
			ref.ref[ha.RHP] += 0.0005
		elif ref.ref[ha.RHP] > LHP_final +.001: 	  
			ref.ref[ha.RHP] -= 0.0005
		else:
			check_five = 1 
		if ref.ref[ha.RAP] < LAP_final -.001:
			ref.ref[ha.RAP] += 0.00025
		elif ref.ref[ha.RAP] > LAP_final +.001: 	  
			ref.ref[ha.RAP] -= 0.00025
		else:
			check_one = 1 
		if ref.ref[ha.LAP] < RAP_final -.001: 
			ref.ref[ha.LAP] += 0.00025
		elif ref.ref[ha.LAP] > RAP_final +.001: 	  
			ref.ref[ha.LAP] -= 0.00025
		else:
			check_two = 1
		r.put(ref)
		time.sleep(.01)

	RAP_final = ref.ref[ha.RAP]
	RKN_final = ref.ref[ha.RKN]
	RHP_final = ref.ref[ha.RHP]
	LAP_final = ref.ref[ha.LAP]  
	LKN_final = ref.ref[ha.LKN] 
	LHP_final = ref.ref[ha.LHP]  
	RAR_final = ref.ref[ha.RAR]
	LAR_final = ref.ref[ha.LAR]
	RHR_final = ref.ref[ha.RHR]
	LHR_final = ref.ref[ha.LHR]
	print("Robot ready-right")

#fix the robot to an initial ready positiion
def get_ready_left():
	time.sleep(5)
	check_one = 0
	check_two = 0
	check_three = 0
	check_four = 0
	check_five = 0
	check_six = 0
	while check_four+check_three+check_five+check_six+check_one+check_two!= 6:
		if ref.ref[ha.LKN] < RKN_final - 0.001:
			ref.ref[ha.LKN] += 0.0005
		elif ref.ref[ha.LKN] > RKN_final + 0.001: 	  
			ref.ref[ha.LKN] -= 0.0005
		else:
			check_four = 1 		
		if ref.ref[ha.RKN] < LKN_final -.001:
			ref.ref[ha.RKN] += 0.0005
		elif ref.ref[ha.RKN] > LKN_final + 0.001: 	  
			ref.ref[ha.RKN] -= 0.0005
		else:
			check_three = 1
		if ref.ref[ha.RHP] < LHP_final -0.001:
			ref.ref[ha.RHP] += 0.0005
		elif ref.ref[ha.RHP] > LHP_final +.001: 	  
			ref.ref[ha.RHP] -= 0.0005
		else:
			check_six = 1 
		if ref.ref[ha.LHP] < RHP_final -.001:
			ref.ref[ha.LHP] += 0.0005
		elif ref.ref[ha.LHP] > RHP_final +.001: 	  
			ref.ref[ha.LHP] -= 0.0005
		else:
			check_five = 1 
		if ref.ref[ha.LAP] < RAP_final -.001:
			ref.ref[ha.LAP] += 0.00025
		elif ref.ref[ha.LAP] > RAP_final +.001: 	  
			ref.ref[ha.LAP] -= 0.00025
		else:
			check_one = 1 
		if ref.ref[ha.RAP] < LAP_final -.001: 
			ref.ref[ha.RAP] += 0.00025
		elif ref.ref[ha.RAP] > LAP_final +.001: 	  
			ref.ref[ha.RAP] -= 0.00025
		else:
			check_two = 1
		r.put(ref)
		time.sleep(.01)

	RAP_final = ref.ref[ha.RAP]
	RKN_final = ref.ref[ha.RKN]
	RHP_final = ref.ref[ha.RHP]
	LAP_final = ref.ref[ha.LAP]  
	LKN_final = ref.ref[ha.LKN] 
	LHP_final = ref.ref[ha.LHP]  
	RAR_final = ref.ref[ha.RAR]
	LAR_final = ref.ref[ha.LAR]
	RHR_final = ref.ref[ha.RHR]
	LHR_final = ref.ref[ha.LHR]
	print("Robot ready - left")
	

def stand():
	ref.ref[ha.RAP] = 0
	ref.ref[ha.LAP] = 0
	ref.ref[ha.LKN] = 0
	ref.ref[ha.RKN] = 0
	ref.ref[ha.RHP] = 0
	ref.ref[ha.LHP] = 0
	r.put(ref)
	time.sleep(0.5)
	print("Robot final stand")
	
	
def main():	
	global step_count
	global step_end

	while step_end > 0:
		if step_count == 0:
			initialize()
			init_ready()
			init_lean()
			init_done()
			step_count = 1
		if step_count == 1:
			step_count = 2
			step_end -= 1
			right_step()
			shift_right_weight()
			get_ready_right()
		if step_count == 2:
			step_count = 1
			step_end -= 1
			left_step()
			shift_left_weight()
			get_ready_left()
	stand()			
	print("Robot - DONE")

main()
