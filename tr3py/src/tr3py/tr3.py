#!/usr/bin/env python

import sys
import time
import signal
import math
import tr3_network
from tr3_joint import Joint

CMD_SET_MODE = 0x10
CMD_SET_POS = 0x11
CMD_RESET_POS = 0x12
CMD_ROTATE = 0x13
CMD_RETURN_STATUS = 0x14
CMD_STOP_RELEASE = 0x15
CMD_STOP_EMERGENCY = 0x16
CMD_FLIP_MOTOR = 0x17
CMD_CALIBRATE = 0x18
CMD_SHUTDOWN = 0x19
CMD_UPDATE_PID = 0x20
CMD_SET_VELOCITY = 0x21

TAU = math.pi * 2.0

close = False
def signal_handler(sig, frame):
    global close
    close = True
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class TR3:
    _msgs = tr3_network.Network()

    _state = None
    state_change = None

    mode_servo = 0x10
    mode_backdrive = 0x11
    mode_rotate = 0x12
    mode_velocity = 0x13

    def __init__(self):
        self.b0 = Joint(self, "b0")
        self.b1 = Joint(self, "b1")
        self.a0 = Joint(self, "a0")
        self.a1 = Joint(self, "a1")
        self.a2 = Joint(self, "a2")
        self.a3 = Joint(self, "a3")
        self.a4 = Joint(self, "a4")
        self.g0 = Joint(self, "g0")
        self.h0 = Joint(self, "h0")
        self.h1 = Joint(self, "h1")
        self.p0 = Joint(self, "p0")
        self.p1 = Joint(self, "p1")
        self.p2 = Joint(self, "p2")
        self._msgs.state_change = self.handle_state_change

        print("TR3 waiting for state")
        while self._state == None:
            self.step()

        self.powerup()
        print("TR3 ready")

    def handle_state_change(self, state):
        if self.state_change != None:
            self.state_change(state)

    def state(self):
        return self._state

    def drive(self, motorLeft, motorRight, motorDuration = 250):
        offsetBinary = 100

        packet = tr3_msgs.Packet()
        packet.address = "b0"
        packet.cmd = CMD_ROTATE
        packet.addParam(int((motorLeft * 100.0) + offsetBinary))
        packet.addParam(int((motorRight * 100.0) + offsetBinary))
        packet.addParam(int(math.floor(motorDuration % 256)))
        packet.addParam(int(math.floor(motorDuration / 256)))

        self._msgs.add(packet)
        self.step()

    def release(self):
        self.a0.release()
        self.a1.release()
        self.a2.release()
        self.a3.release()
        self.a4.release()
        self.h0.release()
        self.h1.release()
        self.b0.release()
        self.b1.release()

    def powerup(self):
        self.p0.setPosition(1)
        self.sleep(2)

    def shutdown(self):
        self.a0.shutdown()
        self.a1.shutdown()
        self.a2.shutdown()
        self.a3.shutdown()
        self.a4.shutdown()
        self.h0.shutdown()
        self.h1.shutdown()
        self.b0.shutdown()
        self.b1.shutdown()
        self.sleep(2)
        self.close()

    def stop(self):
        self.a0.stop()
        self.a1.stop()
        self.a2.stop()
        self.a3.stop()
        self.a4.stop()
        self.h0.stop()
        self.h1.stop()
        self.b0.stop()
        self.b1.stop()

    def setMode(self, mode):
        self.a0.setMode(mode)
        self.a1.setMode(mode)
        self.a2.setMode(mode)
        self.a3.setMode(mode)
        self.a4.setMode(mode)
        self.h0.setMode(mode)
        self.h1.setMode(mode)
        self.b0.setMode(mode)
        self.b1.setMode(mode)

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
        global close
        if close == True:
            self._msgs.close()
            self.close()
            return

        self._msgs.step()
        self._state = self._msgs.state()

        if self._state == None:
            return

        ids, pos, rot, eff, vel, trq = self._state
        for i in range(len(ids)):
            try:
                getattr(self,ids[i])._position = pos[i]
                getattr(self,ids[i])._rotations = rot[i]
                getattr(self,ids[i])._effort = eff[i]
                getattr(self,ids[i])._velocity = vel[i]
                getattr(self,ids[i])._torque = trq[i]
            except:
                pass

        self.step_odom()

    l_pos_prev = None
    r_pos_prev = None
    pos_x = 0
    pos_y = 0
    pos_th = 0

    def step_odom(self):
        wheel_dist = 0.6562

        if (self.b0._position == None or self.b0._rotations == None or self.b1._position == None or self.b1._rotations == None):
            return

        l_pos = -(self.b0._position + self.b0._rotations * 6.283185)
        r_pos = (self.b1._position + self.b1._rotations * 6.283185)

        if (self.l_pos_prev == None or self.r_pos_prev == None):
            self.l_pos_prev = l_pos
            self.r_pos_prev = r_pos
            return
        else:
            d_l = l_pos - self.l_pos_prev
            d_r = r_pos - self.r_pos_prev

            self.l_pos_prev = l_pos
            self.r_pos_prev = r_pos

        dist_per_rad = 0.15875 # meters
        dist_l = d_l * dist_per_rad
        dist_r = d_r * dist_per_rad
        dist_c = (dist_l + dist_r) / 2.0;

        self.pos_x += dist_c * math.cos(self.pos_th)
        self.pos_y += dist_c * math.sin(self.pos_th)
        self.pos_th += (dist_r - dist_l) / wheel_dist

    def spin(self, condition = True):
        global close
        while condition == True or close == True:
            self.step()

    def close(self):
        self.p0.setPosition(0)
        self.sleep(1)
        while self._msgs._msgs != "":
            self.step()
