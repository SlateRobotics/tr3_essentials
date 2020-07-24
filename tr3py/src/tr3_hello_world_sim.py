#!/usr/bin/env python

import time
from tr3py.tr3_sim import TR3

print TR3

tr3 = TR3()
tr3.setMode(tr3.mode_servo)
tr3.release()

poses = [[0,0,0,0,0,0,4],[0,0,0,1.5,0,0,4],[0,0,0.7,1.5,0,1,4],[0,0,-0.7,1.5,0,1,8],[0,0,0,1.5,0,0,4]]

tr3.drive(0.10, -0.10)
for p in poses:
	print("Moving to position", p)
	tr3.a0.setPosition(p[0])
	tr3.a1.setPosition(p[1])
	tr3.a2.setPosition(p[2])
	tr3.a3.setPosition(p[3])
	tr3.a4.setPosition(p[4])
	tr3.g0.setPosition(p[5])
	time.sleep(p[6])

tr3.drive(0, 0)
tr3.stop()
