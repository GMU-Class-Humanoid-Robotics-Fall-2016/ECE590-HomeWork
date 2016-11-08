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
	
def getDegrees(enc):
   DPT = float(300/1024)
   deg = float((enc - 1024/2)*DPT)
   return int(deg)

def getEncoders(deg):
   DPT = float(300.0/1024.0)
   enc = deg/DPT + 512
   return [enc/16 % 16,(enc/16%16)/16]
   
def getTorque(kgfcm):
	dynTorBits = int((torque * MAXBITS)/ MAXTOR)
	dynTorLow = dynTorBits & 0xff
	dynTorHigh = (dynTorBits >> 8) & 0xff
	return [dynTorHigh, dynTorLow]
	
def getVelocity(rpm):
	dynVelBits = int((vel * MAXBITS)/MAXVEL)
	dynVelLow = dynVelBits & 0xff
	dynVelHigh = (dynVelBits >> 8) & 0xff
	return [dynVelHigh, dynVelLow]

def receive_command(servo_id):
	a_head = [0xff, 0xff]
	a_id = [servo_id]
	a_len = [0x04]
	a_cmd = [0x02]
	a_address = [0x24]
	a_read_l = [0x06]
	a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_read_l[0] ) & 0xff]  # get checksum
	out_put= a_head + a_id + a_len + a_cmd + a_address + a_read_l + a_sum

	the_out = bytearray(out_put)
	ser.write(the_out)
	
	input_byte = bytearray(12)
	input_byte2 = [0,0,0,0,0,0,0,0,0,0,0,0]
	
	for i in range(12):
		input_byte[i] = ser.read(1)
		input_byte2[i] =  int(input_byte[i])
		

	######################
	inHead		= [input_byte2[0], input_byte2[1]]
	inId		= input_byte2[2] 
	inLen		= input_byte2[3]
	
	inPos		=  ((input_byte2[6] & 0x03) << 8) + input_byte2[5]
	
	inVelDirection = (input_byte2[8] >> 2)
	inVel		=  ((input_byte2[8] & 0x03) << 8) + input_byte2[7]
	
	inTorDirection = (input_byte2[10] >> 2)
	inTor		= ((input_byte2[10] & 0x03) << 8) + input_byte2[9]
	
	inSum		= input_byte2[11]
	
	if(inVelDirection):
		inVel = inVel * -1.0
	if(inTorDirection):
		inTor = inTor * -1.0
	
	pos = dynPosToDeg(inPos)
	vel = dynVelToRPM(inVel)
	tor = dynTorqueToKGFCM(inTor)
	
	return [inId, pos, vel, tor]
   
  
def perform_command(servo_id,degrees,velocity=None,torque=None):
	vel_h = 0x00
	vel_l = 0xff
	tor_h = 0x00
	tor_l = 0xff
	
	[pos_h,pos_l] = getEncoders(degrees)
	
	if(vel):
		[vel_h,vel_l] = getVelocity(velocity)
		
	if(torque):
		[vel_h,vel_l] = getVelocity(velocity)
		[tor_h,tor_l] = getTorque(torque)
		
	a_head = [0xff, 0xff]
	a_id = [dynId]
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

	if(torque):
		a_len = [0x07]
		a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_goal_l[0] + a_goal_h[0] + a_vel_l[0] + a_vel_h[0] + a_tor_l[0] + a_tor_h[0]) & 0xff]  # get checksum
		out_put = a_head + a_id + a_len + a_cmd + a_address + a_goal_l + a_goal_h + a_vel_l + a_vel_h + a_tor_l + a_tor_h + a_sum
		
	elif(vel):
		a_len = [0x09]
		a_sum = [~(a_id[0] + a_len[0] + a_cmd[0] + a_address[0] + a_goal_l[0] + a_goal_h[0] + a_vel_l[0] + a_vel_h[0]) & 0xff]  # get checksum
		out_put = a_head + a_id + a_len + a_cmd + a_address + a_goal_l + a_goal_h + a_vel_l + a_vel_h + a_sum
    
	the_out_put = bytearray(out_put)
	print(out_put)
	ser.write(the_out_put)
	
	feedback_list = ser.read(6)
	
def feedback(signum, stack):
	for i in range(2):
		[output_data.id, output_data.pos, output_data.vel, output_data.tor] = receive_command(i+2)
		dout.put(output_data)
		print "Sent Current Pos, Vel, Torque"
	
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
	