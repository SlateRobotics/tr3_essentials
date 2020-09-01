#!/usr/bin/env python

import time
import socket
import threading
import rospy
import sys
import signal
import numpy as np
import math
import serial
import datetime
import tr3_msgs
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class Packet:
	address = "x0"
	msgId = 0x00
	cmd = 0x00
	length = 0
	params = []
	checksum = 0

	_startByte = 0xFF

	def __init__(self, addr = "x0"):
		self.address = addr
		self.params = []
		return

	def addParam(self, p):
		self.params.append(p)

	def computeChecksum(self):
		self.checksum = 0
		self.checksum = self.checksum + self.msgId
		self.checksum = self.checksum + self.cmd
		self.checksum = self.checksum + self.length

		for p in self.params:
			self.checksum = self.checksum + int(p)

		self.checksum = int(math.floor(self.checksum % 256))

	def computeLength(self):
		self.length = 4 + len(self.params)

	def toList(self):
		self.computeLength()

		result = [self._startByte, self.address, self.msgId, self.length, self.cmd]

		for p in self.params:
			result.append(p)

		result.append(self.checksum)
		return result

	def toString(self):
		self.computeLength()

		msgString = str(self.address) + ':'
		msgString = msgString + str(self.msgId) + ',0,'
		msgString = msgString + str(self.length) + ','
		msgString = msgString + str(self.cmd) + ','

		for p in self.params:
			msgString = msgString + str(p) + ','

		return msgString + ';'

class Network:
	_msgs = ""
	_msgId = 0

	_freq_hz = 20
	_close = False
	_step_avail = True

	server_socket = None
	server_conn = None
	server_addr = None

	_state = None
	state_change = None

	t_start = datetime.datetime.now()
	t_curr = datetime.datetime.now()

	def __init__(self):
		self._msgs = ""
		self._state_change = self.on_state_change
		self.connect()

	def connect(self):
		if self.server_socket != None:
			self.server_socket.close()

		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.server_socket.bind(('', 12345))
		self.server_socket.listen(5)

		print "Waiting for connection on port 12345"
		self.server_conn, self.server_addr = self.server_socket.accept()
		print 'Got connection from', self.server_addr

	def send(self, d):
		if self._close == True:
			return

		try:
			self.server_conn.send(d.encode())
		except Exception as e:
			print(e)
			print("Socket send failed. Reconnecting...")
			self.connect()

	def recv(self, i):
		if self._close == True:
			return

		try:
			return self.server_conn.recv(i)
		except Exception as e:
			print(e)
			print("Socket recv failed. Reconnecting...")
			self.connect()

	def on_state_change(self, state):
		if self.state_change != None:
			self.state_change(state)

	def incrementMsgId(self):
		if (self._msgId < 255):
			self._msgId = self._msgId + 1
		else:
			self._msgId = 0

	def state(self):
		return self._state

	def add(self, packet):
		packet.msgId = self._msgId
                self.incrementMsgId()
		self._msgs = self._msgs + packet.toString()

	def close(self):
		self._close = True

	def step_socket(self):
		if self._msgs != "":
			#print " -> " + self._msgs
			self.send(self._msgs.encode())
			self._msgs = ""
		else:
			self.send("nc;")

		res = self.recv(4096)
		if res and res != "ns;":
			_states = res.split(";")
			self._state = ([], [], [], [], [], [])
			for _state in _states:
				if (len(_state.split(':')) > 1):
					id = _state.split(':')[0]
					state = _state.split(':')[1]

                                        self._state[0].append(id)

                                        vals = state.split(',');
                                        for idx in range(5):
    					    try:
						state = float(vals[idx])
						self._state[idx + 1].append(state)
					    except:
						self._state[idx + 1].append(0.0)

                if self._state_change != None:
			self._state_change(self._state)

	def step(self):
		if self._close == True:
			return

		self.t_curr = datetime.datetime.now()
		t_delta = (self.t_curr - self.t_start).total_seconds()
		if t_delta > 1.0 / self._freq_hz and self._step_avail == True:
			self._step_avail = False
			self.step_socket()
			self._step_avail = True
			self.t_start = datetime.datetime.now()
