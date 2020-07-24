#!/usr/bin/env python

import sys
import time
import signal
import math

from geometry_msgs.msg import Twist

class go_to_goal:
    priority = 3

    def __init__(self):
        pass

    def flag(self, cloud):
        return True

    def step(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 8.8
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = 0

        return cmd_vel
