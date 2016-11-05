#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
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
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0

RED_R_RANGE = np.array([100, 255])
RED_G_RANGE = np.array([0, 30])
RED_B_RANGE = np.array([0, 30])

GREEN_R_RANGE = np.array([0, 30])
GREEN_G_RANGE = np.array([100, 255])
GREEN_B_RANGE = np.array([0, 30])

BLUE_R_RANGE = np.array([0, 30])
BLUE_G_RANGE = np.array([0, 30])
BLUE_B_RANGE = np.array([100, 255])

def resize(img):
    resize_img = cv2.resize(img, (0,0),fx=0.5,fy=0.5)
    cv2.imshow("resize", resize_img)
    cv2.waitKey(10)
    return resize_img

def to_grayscale(img):
    grayscale_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    cv2.imshow("to_grayscale", grayscale_img)
    cv2.waitKey(10) 
    return grayscale_img
	
def findBalls(img):
	red_lower = np.array([100,0,0],dtype="uint8")
	red_upper = np.array([255,30,30],dtype="uint8")
	green_lower = np.array([0,100,0],dtype="uint8")
	green_upper = np.array([30,255,30],dtype="uint8")
	blue_lower = np.array([0,0,100],dtype="uint8")
	blue_upper = np.array([30,30,255],dtype="uint8")
	
	red_mask = cv2.inRange(img, red_lower, red_upper)
	green_mask = cv2.inRange(img, green_lower, green_upper)
	blue_mask = cv2.inRange(img, blue_lower, blue_upper)
	
	red_cnts = cv2.findContours(red_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	green_cnts = cv2.findContours(green_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	blue_cnts = cv2.findContours(blue_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	
	red_center = None
	green_center = None
	blue_center = None
	
	if len(red_cnts) > 0:
		c = max(red_cnts,key=cv2.contourArea)
		((x,y),radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		red_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		print"red center %d %d" %(red_center[0],red_center[1])
	
		cv2.circle(img, (int(x),int(y)), 5,(0,0,0),2)
		#cv2.circle(img,red_center,5,(0,0,255),-1)
	
	elif len(green_cnts) > 0:
		c = max(green_cnts,key=cv2.contourArea)
		((x,y),radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		green_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		print "green center %d %d" %(green_center[0],green_center[1])
		
		cv2.circle(img, (int(x),int(y)), 5,(0,0,0),2)
		#cv2.circle(img,green_center,5,(0,0,255),-1)
		
	elif len(blue_cnts) > 0:
		c = max(blue_cnts,key=cv2.contourArea)
		((x,y),radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		blue_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		print "blue center %d %d" %(blue_center[0],blue_center[1])
		
		cv2.circle(img, (int(x),int(y)), 5,(0,0,0),2)
		#cv2.circle(img,blue_center,5,(0,0,255),-1)
	i = 100
	cv2.imshow("findBalls",img)
	cv2.waitKey(10)
	 
def dilate(img):
	kernel = np.ones((10,10),np.uint8)
	final_dialation = cv2.dilate(img, kernel, iterations=1)
	cv2.imshow("dilate", final_dialation)
	cv2.waitKey(10)
	return
	
def erode(img):
	kernel = np.ones((10,10),np.uint8)
	final_erosion = cv2.erode(img,kernel,iterations=1)
	cv2.imshow("erode", final_erosion)
	cv2.waitKey(10)
	return

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl", img)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format
    if(img.ndim != 0):
		if(i < 150):
			resizeImg = resize(img)
		elif (i <300):
			cv2.destroyWindow("resize")
			findBalls(img)
		elif (i < 450):
			cv2.destroyWindow("findBalls")
			grayScaleImg = to_grayscale(img)
		elif (i < 600):
			cv2.destroyWindow("to_grayscale")
			erode(img)
		elif (i < 750):
			cv2.destroyWindow("erode")
			dilate(img)			
		else:
			cv2.destroyWindow("dilate")
			print("done")
			break
		ref.ref[0] = -0.5
		ref.ref[1] = 0.5

		#print 'Sim Time = ', tim.sim[0]
	    
		# Sets reference to robot
		r.put(ref)
		print(i)
		i+=1
		# Sleeps
		time.sleep(0.1)   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
