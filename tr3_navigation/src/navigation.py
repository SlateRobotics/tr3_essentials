#!/usr/bin/env python

import time
import rospy
import sys
import signal
from sensor_msgs.msg import PointCloud2
from tr3py.tr3_sim import TR3
from tr3_navigation import TR3_Nav

tr3 = TR3()
tr3_nav = None

processing = False

def signal_handler(sig, frame):
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def camera_depth_callback(msg):
	global processing

	if (processing == False):
		processing = True
		time_start = time.clock()

		cmd_vel = tr3_nav.step(msg)
		tr3.drive(cmd_vel)

		tr3.h0.setPosition(0.0)
		tr3.h1.setPosition(-0.2)

		time_elapsed = (time.clock() - time_start)
		processing = False

if __name__ == '__main__':
	rospy.Subscriber("/camera/depth/points", PointCloud2, camera_depth_callback)
	tr3_nav = TR3_Nav()
	tr3.spin()
