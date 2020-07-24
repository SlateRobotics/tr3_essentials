#!/usr/bin/env python

import curses
import time
import rospy
import sys
import signal
import numpy as np
import math
import datetime
from tr3py.tr3 import TR3

tr3 = TR3()
tr3.setMode(tr3.mode_rotate)
tr3.release()

jointSelected = 0;
joints = ["b0", "b1", "a0", "a1", "a2", "a3", "a4", "g0", "h0", "h1"]

modeSelected = 0
modes = ["ROTATE","BACKDRIVE","SERVO"]

stop = "FALSE"
speed = 100

def program_actuator():
    global jointSelected, joints, modeSelected, modes, stop, tr3, speed

    try:
        while True:
            char = screen.getch()

            if char == ord(' '): # emergency stop
                if stop == "FALSE":
                    stop = "TRUE"
                    tr3.stop()
                else:
                    stop = "FALSE"
                    tr3.release()
            elif char == ord('q'):
                tr3.shutdown()
                break
            elif char == ord('p'):
                program_drive()
                break
            elif char == ord('h'):
                getattr(tr3, aid).setMode(TR3.mode_servo)
                getattr(tr3, aid).setPosition(0.0)
            elif char == ord('c'): getattr(tr3, aid).calibrate()
            elif char == ord('r'): getattr(tr3, aid).resetEncoderPosition()
            elif char == ord('f'): getattr(tr3, aid).flipMotor()
            elif char == ord('b'): getattr(tr3, aid).setMode(TR3.mode_backdrive)
            elif char == ord('n'): getattr(tr3, aid).setMode(TR3.mode_rotate)
            elif char == ord('m'): getattr(tr3, aid).setMode(TR3.mode_servo)
            elif char == ord('d'): modeSelected = modeSelected + 1
            elif char == ord('a'): modeSelected = modeSelected - 1
            elif char == ord('w'): speed = speed + 5
            elif char == ord('s'): speed = speed -5
            elif char == curses.KEY_UP: jointSelected = jointSelected + 1
            elif char == curses.KEY_DOWN: jointSelected = jointSelected - 1
            elif char == curses.KEY_LEFT: cmd = -1
            elif char == curses.KEY_RIGHT: cmd = 1
            else:
                    cmd = 0

            aid = joints[jointSelected]

            if modeSelected != prevMode:
                if modeSelected == 0:
                    getattr(tr3, aid).setMode(tr3.mode_rotate)
                elif modeSelected == 1:
                    getattr(tr3, aid).setMode(tr3.mode_backdrive)
                elif modeSelected == 2:
                    getattr(tr3, aid).setMode(tr3.mode_servo)

            if modeSelected == 0:
                if (cmd != 0):
                    getattr(tr3, aid).actuate(cmd, speed)
            elif modeSelected == 2:
                    getattr(tr3, aid).setPosition(cmd, speed)

            tr3.step()

            screen.clrtobot()
            screen.addstr(0,0,"Welcome. Use arrow keys to control. Press 'Q' to exit, SPACE to emergency stop the robot.")
            screen.addstr(1,0,"ACTUATOR: " + aid)
            screen.addstr(2,0,"MODE: " + modes[modeSelected])
            screen.addstr(3,0,"CMD: " + str(cmd) + "   ")
            screen.addstr(4,0,"STOP: " + stop)
            screen.addstr(5,0,"INPUT: " + str(char))
            screen.addstr(6,0,"SPEED: " + str(speed))
            screen.refresh()

            curses.flushinp()
            time.sleep(0.05)
    finally:
            screen.addstr(0,0,"exit")
            curses.nocbreak()
            screen.keypad(0)
            curses.echo()
            curses.endwin()

def program_drive():
    global jointSelected, joints, modeSelected, modes, stop, tr3, speed

    s = 0.60

    try:
        while True:
            char = screen.getch()

            if char == ord(' '): # emergency stop
                if stop == "FALSE":
                    stop = "TRUE"
                    tr3.stop()
                else:
                    stop = "FALSE"
                    tr3.release()
            elif char == ord('q'):
                tr3.shutdown()
                break
            elif char == ord('p'):	
                program_actuator()
                break
            elif char == ord('d'): modeSelected = modeSelected + 1
            elif char == ord('a'): modeSelected = modeSelected - 1
            elif char == curses.KEY_UP:
                tr3.b0.actuate(-s, 100)
                tr3.b1.actuate(s, 100)
            elif char == curses.KEY_DOWN:
                tr3.b0.actuate(s, 100)
                tr3.b1.actuate(-s, 100)
            elif char == curses.KEY_LEFT:
                tr3.b0.actuate(-s, 100)
                tr3.b1.actuate(-s, 100)
            elif char == curses.KEY_RIGHT:
                tr3.b0.actuate(s, 100)
                tr3.b1.actuate(s, 100)
            else:
                tr3.b0.actuate(0, 100)
                tr3.b1.actuate(0, 100)

            aid = joints[jointSelected]
            tr3.step()

            screen.clrtobot()
            screen.addstr(0,0,"Drive mode.  Use arrow keys to control. Press 'Q' to exit, SPACE to emergency stop the robot.")
            screen.addstr(1,0,"ACTUATOR: " + aid)
            screen.addstr(2,0,"MODE: " + modes[modeSelected])
            screen.addstr(3,0,"CMD: " + str(cmd) + "   ")
            screen.addstr(4,0,"STOP: " + stop)
            screen.addstr(5,0,"INPUT: " + str(char))
            screen.addstr(6,0,"SPEED: " + str(speed))
            screen.refresh()

            curses.flushinp()
            time.sleep(0.05)
    finally:
        screen.addstr(0,0,"exit")
        curses.nocbreak()
        screen.keypad(0)
        curses.echo()
        curses.endwin()

if __name__ == '__main__':
    rospy.init_node('tr3_keyboard_teleop', anonymous=True)

    screen = curses.initscr()
    curses.noecho()
    curses.cbreak()
    screen.keypad(True)
    screen.nodelay(1)

    cmd = 0
    prevMode = modeSelected
    program_actuator()
