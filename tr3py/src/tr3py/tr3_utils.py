#!/usr/bin/env python

import time
import rospy
import sys
import signal
import numpy as np
import math
import datetime
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

# input: numpy array, -1 to 1 value
def getMotorValues(y, rotationStrength):
	desiredMagnitude = abs(y)
	if (abs(rotationStrength) > desiredMagnitude):
		desiredMagnitude = abs(rotationStrength)
		
	if (desiredMagnitude == 0):
		return 0, 0

	# define motors
	left = y
	right = y
	
	#manipulate values based on rotation from rotationStrength
	#rotationStrength = rotationStrength * 3
	left = left + rotationStrength
	right = right - rotationStrength

	# get output magnitude
	outputMagnitude = abs(right)
	if abs(left) > outputMagnitude:
		outputMagnitude = abs(left)

	# scale output to match desired magnitude
	scale = desiredMagnitude / outputMagnitude

	left = left * scale
	right = right * scale

	scaledOutput = (left, right)

	return scaledOutput
			
