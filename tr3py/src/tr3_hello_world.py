#!/usr/bin/env python

from tr3py.tr3 import TR3

tr3 = TR3()
tr3.setMode(tr3.mode_servo)
tr3.release()

a0_state = tr3.a0.state()
a2_state = tr3.a2.state()
a4_state = tr3.a4.state()

tr3.a0.setPosition(a0_state + 0.25)
tr3.a2.setPosition(a2_state + 0.48)
tr3.a4.setPosition(a4_state - 0.15)
tr3.sleep(5)

tr3.a0.setPosition(a0_state)
tr3.a2.setPosition(a2_state)
tr3.a4.setPosition(a4_state)
tr3.sleep(5)

tr3.stop()
tr3.close()
