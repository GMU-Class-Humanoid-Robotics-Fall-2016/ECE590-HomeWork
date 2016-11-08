import hubo_ach as ha
import ach
from ctypes import *

class SERVODATA(Structure):
    _pack_ = 1
    _fields_ = [("id"	, c_int8),
				("pos"	, c_double),
				("vel"	, c_double),
				("tor"	, c_double)
				 ]

dout = ach.Channel('servo_controller')
din  = ach.Channel('servo_commander')

input_data = SERVODATA()
output_data = SERVODATA()

din.flush()

while 1 :
	[statuss, framesizes] = din.get(input_data, wait=True, last=True)
	
	output_data.id  = 3
	output_data.pos = 90.0
	output_data.vel = 75.0
	output_data.tor = 16.0
	dout.put(output_data)
	
	time.sleep(10)
	
	output_data.id  = 2
	output_data.pos = 30.0
	output_data.vel = 60.0
	output_data.tor = 11.0
	dout.put(output_data)
	
	time.sleep(10)
