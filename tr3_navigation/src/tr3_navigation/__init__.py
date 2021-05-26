#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math
import numpy as np

from geometry_msgs.msg import Twist

from .cloud_transformer import cloud_transformer
from .routine_collision import collision
from .routine_close_call import close_call
from .routine_avoid_obstacle import avoid_obstacle
from .routine_go_to_goal import go_to_goal

def signal_handler(sig, frame):
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class TR3_Nav:
    routines = []
    transformer = None

    maxWheelVelocity = 8.8
    wheelVelocityDesired = [0, 0] # b0, b1
    travelDesired = [0, 0] # velocity, omega

    def __init__(self):
        self.transformer = cloud_transformer()
        self.addRoutine(collision())
        self.addRoutine(close_call())
        self.addRoutine(avoid_obstacle())
        self.addRoutine(go_to_goal())

    # args: routine
    def addRoutine(self, r):
        self.routines.append(r)
        self.routines.sort(key=lambda x: x.priority, reverse=True)

    # args: PointCloud2 in camera_link_optical frame
    # returns: list, (b0, b1) desired velocities
    def step(self, c):
        cloud = self.transformer.transform(c)
        for r in self.routines:
            if r.flag(cloud) == True:
            	self.travelDesired = r.step()

        #self.computeVelocity()
        #return self.wheelVelocityDesired

        #print self.travelDesired
        return self.travelDesired

    def computeVelocity(self):
        velocity = self.travelDesired[0]
        omega = self.travelDesired[1]

    	max_vel = 8.8
    	wheel_radius = 0.200
    	wheel_width = 0.6038

        b0_vel = velocity + (omega * wheel_width / 2)
        b1_vel = velocity - (omega * wheel_width / 2)

        self.wheelVelocityDesired = (b0_vel, b1_vel)