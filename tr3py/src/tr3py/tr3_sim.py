#!/usr/bin/env python

import sys
import time
import signal
import math
import rospy
from tr3_joint_sim import Joint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Float64

class TR3:
	_state = None
	state_change = None

	mode_servo = 0x10
	mode_backdrive = 0x11
	mode_rotate = 0x12

	base = None

	def __init__(self):
		rospy.init_node('tr3_sim', anonymous=True)
		rospy.Subscriber("/tr3/state", JointState, self._cbtr3state)

		self.base = rospy.Publisher("/tr3/base/diff/cmd_vel", Twist, queue_size=10);

		self.a0 = Joint(self, "a0")
		self.a1 = Joint(self, "a1")
		self.a2 = Joint(self, "a2")
		self.a3 = Joint(self, "a3")
		self.a4 = Joint(self, "a4")
		self.g0 = Joint(self, "g0")
		self.h0 = Joint(self, "h0")
		self.h1 = Joint(self, "h1")

		print("TR3 waiting for ROS state on /tr3/state")
		time.sleep(2)
		while self._state == None:
			pass

		print("TR3 ready")

	def _cbtr3state(self, msg):
		state = [msg.name, []]
		for i in range(len(state[0])):
			id = msg.name[i]
			pos = msg.position[i]
			state[1].append(msg.position[i])

			try:
				getattr(self,id)._state = pos
			except:
				pass

		self._state = state

		if self.state_change != None:
			self.state_change(self._state)

	def state(self):
		return self._state

	def drive(self, twist):
		self.base.publish(twist)

	def release(self):
		self.a0.release()
		self.a1.release()
		self.a2.release()
		self.a3.release()
		self.a4.release()
		self.h0.release()
		self.h1.release()

	def stop(self):
		self.a0.stop()
		self.a1.stop()
		self.a2.stop()
		self.a3.stop()
		self.a4.stop()
		self.h0.stop()
		self.h1.stop()

	def setMode(self, mode):
		self.a0.setMode(mode)
		self.a1.setMode(mode)
		self.a2.setMode(mode)
		self.a3.setMode(mode)
		self.a4.setMode(mode)
		self.h0.setMode(mode)
		self.h1.setMode(mode)

	def sleep(self, sec):
		t_start = time.time()
		while time.time() - t_start < sec:
			self.step()

	def getJoint(self, id):
		try:
			return getattr(self,id)
		except:
			pass

	def step(self):
		pass

	def spin(self, condition = True):
		global close
		print "TR3 Ready"
		while condition == True or close == True:
			self.step()
		self.close()

	def close(self):
		pass
