import time
import serial
import signal

import hubo_ach as ha
import ach
from ctypes import *

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=1000000,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.open()
ser.isOpen()

class SERVO_DATA(Structure):
    _pack_ = 1
    _fields_ = [("id"	, c_int8),
				("pos"	, c_double),
				("vel"	, c_double),
				("tor"	, c_double)]
	

def getEncPos(deg):
	pos_bits = int(( (deg + 150.0)*1024.0 )/300.0)
	pos_low = pos_bits & 0xff
	pos_high = (pos_bits >> 8) & 0xff
	return [pos_high, pos_low]

def getDegrees(encpos):
	return (encpos*300.0/1024.0) - 150.0
	
def getEncVel(vel):
	vel_bits = int((vel * 1024.0)/114.0)
	vel_low = vel_bits & 0xff
	vel_high = (vel_bits >> 8) & 0xff
	return [vel_high, vel_low]
	
def getVelocity(encvel):
	return (encvel*114.0/1024.0)
	
def getEncTor(tor):
	tor_bits = int((tor * 1024.0)/ 16.5)
	tor_low = tor_bits & 0xff
	tor_high = (tor_bits >> 8) & 0xff
	return [tor_high, tor_low]
	
def getTorque(enctor):
	return (enctor*16.5/1024.0)

def receive_command(servo_id):
	a_head = [0xff, 0xff]
	a_id = [servo_id & 0xff]
	a_len = [0x04]
	a_cmd = [0x02]
	a_address = [0x24]
	a_read_l = [0x06]
	a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_read_l[0] ) & 0xff]
	out_put= a_head + a_id + a_len + a_cmd + a_address + a_read_l + a_sum

	the_out = bytearray(out_put)
	ser.write(the_out)
	
	input_byte = bytearray(12)
	input_byte2 = [0]*12
	
	for i in range(12):
		input_byte[i] = ser.read(1)
		input_byte2[i] =  int(input_byte[i])
		
	b_Head = [input_byte2[0], input_byte2[1]]
	b_Id = input_byte2[2] 
	b_Len = input_byte2[3]
	b_Pos =  ((input_byte2[6] & 0x03) << 8) + input_byte2[5]
	b_vel_direc = (input_byte2[8] >> 2)
	b_Vel =  ((input_byte2[8] & 0x03) << 8) + input_byte2[7]
	b_tor_direc = (input_byte2[10] >> 2)
	b_Tor = ((input_byte2[10] & 0x03) << 8) + input_byte2[9]
	b_Sum = input_byte2[11]
	
	if(b_vel_direc):
		b_Vel = b_Vel * -1.0
	if(b_tor_direc):
		b_Tor = b_Tor * -1.0
	
	pos = getEncPos(b_Pos)
	vel = getEncVel(b_Vel)
	tor = getEncTor(b_Tor)
	
	return [b_Id, pos, vel, tor]
   
  
def perform_command(servo_id,degrees,velocity=None,torque=None):
	vel_h = 0x00
	vel_l = 0xff
	tor_h = 0x00
	tor_l = 0xff
	
	[pos_h,pos_l] = getEncPos(degrees)
	
	if(velocity != None):
		[vel_h,vel_l] = getEncVel(velocity)
		
	if(torque != None):
		[vel_h,vel_l] = getEncVel(velocity)
		[tor_h,tor_l] = getEncTor(torque)
		
	a_head = [0xff, 0xff]
	a_id = [servo_id & 0xff]
	a_len = [0x05]
	a_cmd = [0x03]
	a_address = [0x1e]
	a_goal_l = [pos_l]
	a_goal_h = [pos_h]
	a_vel_l	= [vel_l] 
	a_vel_h	= [vel_h]
	a_tor_l	= [tor_l]
	a_tor_h	= [tor_h]
	
	a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_goal_l[0] + a_goal_h[0]) & 0xff]
	out_put = a_head + a_id + a_len + a_cmd + a_address + a_goal_l + a_goal_h + a_sum

	if(torque != None):
		a_len = [0x07]
		a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_goal_l[0] + a_goal_h[0] + a_vel_l[0] + a_vel_h[0] + a_tor_l[0] + a_tor_h[0]) & 0xff]  # get checksum
		out_put = a_head + a_id + a_len + a_cmd + a_address + a_goal_l + a_goal_h + a_vel_l + a_vel_h + a_tor_l + a_tor_h + a_sum
		
	elif(velocity != None):
		a_len = [0x09]
		a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_goal_l[0] + a_goal_h[0] + a_vel_l[0] + a_vel_h[0]) & 0xff]  # get checksum
		out_put = a_head + a_id + a_len + a_cmd + a_address + a_goal_l + a_goal_h + a_vel_l + a_vel_h + a_sum
    
	the_out_put = bytearray(out_put)
	ser.write(the_out_put)
		
def feedback(signum, stack):
	for i in range(0,2):
		[output_data.id, output_data.pos, output_data.vel, output_data.tor] = receive_command(i+2)
		dout.put(output_data)
	
din = ach.Channel('servo_controller')
dout = ach.Channel('servo_commander')

input_data = SERVO_DATA()
output_data = SERVO_DATA()

signal.signal(signal.SIGALRM, feedback)
signal.setitimer(signal.ITIMER_REAL, 0.2, 0.2)

din.flush()

while 1 :
	[statuss, framesizes] = din.get(input_data, wait=False, last=False)
	perform_command(input_data.id, input_data.pos, input_data.vel, input_data.tor)
	