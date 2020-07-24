#!/usr/bin/env python

import sys
import time
import signal
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Float64

class Joint:
	_tr3 = None
	_id = ""
	_state = None

	_pub_pos = None
	_pub_effort = None
	_pub_stop = None
	_pub_mode = None

	def __init__(self, t, i):
		self._tr3 = t
		self._id = i

		_topic = "/tr3/joints/" + self._id;
		self._pub_stop = rospy.Publisher(_topic + "/stop", Bool, queue_size=10)
		self._pub_mode = rospy.Publisher(_topic + "/mode", UInt8, queue_size=10)
		self._pub_pos = rospy.Publisher(_topic + "/control/position", Float64, queue_size=10)
		self._pub_vel = rospy.Publisher(_topic + "/control/velocity", Float64, queue_size=10)
		self._pub_effort = rospy.Publisher(_topic + "/control/effort", Float64, queue_size=10)

	def state(self):
		return self._state

	def release(self):
		self._pub_stop.publish(0)

	def actuate(self, m, motorDuration = 250):
		if m > 1.0:
			m = 1.0
		elif m < -1.0:
			m = -1.0

		self._pub_effort.publish(m * 100.0)

	def setPosition(self, p):
		self._pub_pos.publish(p)

	def setVelocity(self, v):
		self._pub_vel.publish(v)

	def stop(self):
		self._pub_stop.publish(1)

	def setMode(self, mode):
		m = 0
		if (mode == TR3.mode_backdrive):
			m = 1
		if (mode == TR3.mode_servo):
			m = 2

		self._pub_mode.publish(m)
