import hubo_ach as ha
import ach
import sys
import time
from ctypes import *

import math
import numpy as np

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=True)

def simSleep(sec, s, state):
	tick = state.time;
	dt = 0;
	while(dt <= sec):
		s.get(state, wait=False, last=True)
		dt = state.time - tick;
	return

def assign_thetas(parameter,left_right):
	thetas = [0,0,0,0]
	for i in range(0,4):
		theta = parameter[i,1]
		thetas[i] = theta
	#print(thetas)	
	if(left_right == 0):
		ref.ref[ha.LSP] = -1 * thetas[0]
		ref.ref[ha.LEB] = -1 * thetas[1]
		ref.ref[ha.LWP] = -1 * thetas[2]
		ref.ref[ha.LSR] = -1 * thetas[3]
	else:
		ref.ref[ha.RSP] = -1 * thetas[0]
		ref.ref[ha.REB] = -1 * thetas[1]
		ref.ref[ha.RWP] = -1 * thetas[2]
		ref.ref[ha.RSR] = -1 * thetas[3]
	
	r.put(ref)

def transformCalculate(parameter):
	d = parameter[0]
	theta = parameter[1]
	r = parameter[2]
	alpha = parameter[3]
	
	trans_ind = np.array([[np.cos(theta), -1*np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), r*np.cos(theta)],[np.sin(theta), np.cos(theta)*np.cos(alpha), -1*np.cos(theta)*np.sin(alpha), r*np.sin(theta)],[0,np.sin(alpha),np.cos(alpha), d],[0,0,0,1]])
	return trans_ind

def forward_kinematics(parameters):
	joints = 4		#4 dof
	dimension = 3	#x,y,z
	
	Transform = np.eye(dimension+1)
	for i in range(0,joints):
		Transform = Transform.dot(transformCalculate(parameters[i,:]))
	e_homogenous = Transform.dot(np.array([[0.],[0.],[0.],[1.]]))
	e = e_homogenous[0:3,0]
	
	return e
	
def inverse_kinematics(e, parameters):		
	a_lambda = 100
	dimension = 3
	
	parameters[:,1] = np.array([0,0,0,0])	
	print(parameters)
	
	initial_position = np.empty_like(e)
	initial_position[:] = e
	intended_position = np.empty_like(e)
	intended_position[:] = e
	final_position = forward_kinematics(parameters)
	iterval = 0
	while(np.sqrt((intended_position-final_position).conj().transpose().dot((intended_position-final_position))) > math.exp(-6)):
		Jacobian = np.zeros((3,max(np.shape(parameters[:,0]))))
		joints = max(np.shape(parameters[:,0]))
		parameters_new = np.empty_like(
		parameters)
		parameters_new[:] = parameters
		
		for i in range (0, joints):
			parameters_new[i,1] = parameters[i,1] - 0.01
			Jacobian[:,i] = (forward_kinematics(parameters) - forward_kinematics(parameters_new))/0.01
			parameters_new[:] = parameters
		
		J = np.empty_like(Jacobian)
		J[:] = Jacobian
		a = J.dot(J.conj().transpose()) + a_lambda * np.eye(dimension)
		b = intended_position - forward_kinematics(parameters)
		a_size = np.shape(a)

		if (a_size[0] == a_size[1]):
			a_b = np.linalg.solve(a,b)
		else:
			a_b = np.linalg.lstsq(a,b)
		parameters[:,1] = parameters[:,1] + J.conj().transpose().dot(a_b)

		initial_position = final_position
		final_position = forward_kinematics(parameters)
		iterval = iterval + 1
		dis = np.sqrt((intended_position-final_position).conj().transpose().dot((intended_position-final_position)))
		#print(dis)
		if(iterval == 50000):
			print("error")
			return parameters
	return parameters

initial_params = np.array([[0,0,1.79,math.pi/2],[0,0,1.82,0],[0,0,0.6,0],[0,0,0.2,0]])
#assign_thetas(initial_params)
init_e = forward_kinematics(initial_params)
print(init_e)

left_goal_one = np.array([0,4.41,0])
params = inverse_kinematics(left_goal_one, initial_params)
e_check = forward_kinematics(params)
print(e_check)
assign_thetas(params,0)
assign_thetas(params,1)
time.sleep(10)

left_goal_two = np.array([0,3.2,-1.2])
right_goal_two = np.array([0,3.2,-1.2])
next_params_l = inverse_kinematics(left_goal_two, initial_params)
next_params_r = inverse_kinematics(right_goal_two, initial_params)
e_check = forward_kinematics(next_params_l)
print(e_check)
assign_thetas(next_params_l,0)
assign_thetas(next_params_r,1)
time.sleep(10)

left_goal_three = np.array([0,3.6,0.8])
next_params = inverse_kinematics(left_goal_three, initial_params)
e_check = forward_kinematics(next_params)
print(e_check)
assign_thetas(next_params,0)
assign_thetas(next_params,1)
time.sleep(10)

left_goal_four = np.array([0,4.41,0])
next_params = inverse_kinematics(left_goal_four, initial_params)
e_check = forward_kinematics(next_params)
print(e_check)
assign_thetas(next_params,0)
assign_thetas(next_params,1)
time.sleep(10)
