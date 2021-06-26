#!/usr/bin/env python

import time
import os
import math
import yaml
import io
import struct
import datetime

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tr3_msgs.msg import ActuatorState
from tr3_msgs.msg import ActuatorPositionCommand

MODE_STOP = 0
MODE_SERVO = 1
MODE_VELOCITY = 2
MODE_TORQUE = 3
MODE_ROTATE = 4
MODE_BACKDRIVE = 5
MODE_CALIBRATE = 6
MODE_UPDATE_FIRMWARE = 7

class Joint:
    _tr3 = None
    _id = None
    
    _state = None
    _state_received = None

    # tracks the current command for /send_commands
    _command_mode = None
    _command_msg = None
    _command_ts = None

    _limits = [-6.28, 6.28, -0.942, 0.942, -60, 60]
    _pid_pos = [9.000, 0.000, 0.000]
    _pid_vel = [4.000, 4.000, 0.000]
    _pid_trq = [0.300, 0.050, 0.000]

    _pub_stop = None
    _pub_shutdown = None
    _pub_mode = None
    _pub_reset_pos = None
    _pub_reset_trq = None
    _pub_flip = None
    _pub_pid_pos_set = None
    _pub_pid_vel_set = None
    _pub_pid_trq_set = None
    _pub_control_pos = None
    _pub_control_vel = None
    _pub_control_trq = None
    _pub_control_vol = None
    _pub_ota_start = None
    _pub_ota_data = None
    _pub_ota_end = None

    def __init__(self, t, i):
        self._tr3 = t
        self._id = i

        self._pub_stop = rospy.Publisher("/tr3/" + self._id + "/stop", Bool, queue_size=1)
        self._pub_shutdown = rospy.Publisher("/tr3/" + self._id + "/shutdown", Bool, queue_size=1)
        self._pub_mode = rospy.Publisher("/tr3/" + self._id + "/mode", UInt8, queue_size=1)
        self._pub_reset_pos = rospy.Publisher("/tr3/" + self._id + "/reset/position", Bool, queue_size=1)
        self._pub_reset_trq = rospy.Publisher("/tr3/" + self._id + "/reset/torque", Bool, queue_size=1)
        self._pub_flip = rospy.Publisher("/tr3/" + self._id + "/flip", Bool, queue_size=1)
        self._pub_pid_pos_set = rospy.Publisher("/tr3/" + self._id + "/pid_pos/set", Float32MultiArray, queue_size=1)
        self._pub_pid_vel_set = rospy.Publisher("/tr3/" + self._id + "/pid_vel/set", Float32MultiArray, queue_size=1)
        self._pub_pid_trq_set = rospy.Publisher("/tr3/" + self._id + "/pid_trq/set", Float32MultiArray, queue_size=1)
        self._pub_control_pos = rospy.Publisher("/tr3/" + self._id + "/control/position", ActuatorPositionCommand, queue_size=1)
        self._pub_control_vel = rospy.Publisher("/tr3/" + self._id + "/control/velocity", Float64, queue_size=1)
        self._pub_control_trq = rospy.Publisher("/tr3/" + self._id + "/control/torque", Float64, queue_size=1)
        self._pub_control_vol = rospy.Publisher("/tr3/" + self._id + "/control/voltage", Float64, queue_size=1)
        self._pub_ota_start = rospy.Publisher("/tr3/" + self._id + "/ota/start", UInt32, queue_size=1)
        self._pub_ota_data = rospy.Publisher("/tr3/" + self._id + "/ota/data", UInt8MultiArray, queue_size=1)
        self._pub_ota_end = rospy.Publisher("/tr3/" + self._id + "/ota/end", Bool, queue_size=1)
        
        rospy.Subscriber("/tr3/" + self._id + "/state", ActuatorState, self._sub_state)
        rospy.Subscriber("/tr3/" + self._id + "/limit", Float32MultiArray, self._sub_limits)
        rospy.Subscriber("/tr3/" + self._id + "/pid_pos", Float32MultiArray, self._sub_pid_pos)
        rospy.Subscriber("/tr3/" + self._id + "/pid_vel", Float32MultiArray, self._sub_pid_vel)
        rospy.Subscriber("/tr3/" + self._id + "/pid_trq", Float32MultiArray, self._sub_pid_trq)
        
        # we use these to keep track of what the current actuator state should be
        # actuators call /send_commands to request current state in event of crash/error
        rospy.Subscriber("/tr3/" + self._id + "/stop", Bool, self._sub_stop)
        rospy.Subscriber("/tr3/" + self._id + "/mode", UInt8, self._sub_mode)
        rospy.Subscriber("/tr3/" + self._id + "/control/position", ActuatorPositionCommand, self._sub_control_pos)
        rospy.Subscriber("/tr3/" + self._id + "/control/velocity", Float64, self._sub_control_vel)
        rospy.Subscriber("/tr3/" + self._id + "/control/torque", Float64, self._sub_control_trq)
        rospy.Subscriber("/tr3/" + self._id + "/control/voltage", Float64, self._sub_control_vol)
        rospy.Subscriber("/tr3/" + self._id + "/send_commands", Bool, self._sub_send_commands)

    def _sub_state(self, msg):
        self._state_received = datetime.datetime.now()
        self._state = msg

    def _sub_limits(self, msg):
        self._limits = msg.data

    def _sub_pid_pos(self, msg):
        self._pid_pos = msg.data

    def _sub_pid_vel(self, msg):
        self._pid_vel = msg.data

    def _sub_pid_trq(self, msg):
        self._pid_trq = msg.data

    def _sub_stop(self, msg):
        self._command_mode = MODE_STOP
        self._command_msg = msg
        self._command_ts = datetime.datetime.now()

    def _sub_mode(self, msg):
        self._command_mode = "mode"
        self._command_msg = msg
        self._command_ts = datetime.datetime.now()

    def _sub_control_pos(self, msg):
        if (self._command_mode != MODE_STOP):
            self._command_mode = MODE_SERVO
            self._command_msg = msg
            self._command_ts = datetime.datetime.now()

    def _sub_control_vel(self, msg):
        if (self._command_mode != MODE_STOP):
            self._command_mode = MODE_VELOCITY
            self._command_msg = msg
            self._command_ts = datetime.datetime.now()

    def _sub_control_trq(self, msg):
        if (self._command_mode != MODE_STOP):
            self._command_mode = MODE_TORQUE
            self._command_msg = msg
            self._command_ts = datetime.datetime.now()

    def _sub_control_vol(self, msg):
        if (self._command_mode != MODE_STOP):
            self._command_mode = MODE_ROTATE
            self._command_msg = msg
            self._command_ts = datetime.datetime.now()

    def _sub_send_commands(self, msg):
        if msg.data != True:
            return

        if self._command_mode == "mode":
            self._pub_mode.publish(self._command_msg)
        elif self._command_mode == MODE_STOP:
            self._pub_stop.publish(self._command_msg)
        elif self._command_mode == MODE_ROTATE:
            self._pub_control_vol.publish(self._command_msg)
        elif self._command_mode == MODE_VELOCITY:
            self._pub_control_vel.publish(self._command_msg)
        elif self._command_mode == MODE_TORQUE:
            self._pub_control_trq.publish(self._command_msg)
        elif self._command_mode == MODE_SERVO:
            _msg = self._command_msg
            _msg.duration = int(((self._command_ts + datetime.timedelta(milliseconds=_msg.duration)) - datetime.datetime.now()).total_seconds() * 1000.0)
            if _msg.duration < 0:
                _msg.duration = 0
            self._pub_control_pos.publish(_msg)

    def state(self):
        if self._state == None:
            return None

        now = datetime.datetime.now()
        expires = self._state_received + datetime.timedelta(milliseconds=500)

        # state expires after 500ms and will return None
        if now < expires:
            return self._state
        else:
            return None

    def setPosition(self, pos, dur = 0):
        cmd = ActuatorPositionCommand()
        cmd.position = pos
        cmd.duration = dur
        self._pub_control_pos.publish(cmd)

    def setVelocity (self, vel):
        self._pub_control_vel.publish(vel)

    def setTorque (self, trq):
        self._pub_control_trq.publish(trq)

    def setVoltage (self, vol):
        self._pub_control_vol.publish(vol)

    def release(self):
        self._pub_stop.publish(0)

    def stop(self):
        self._pub_stop.publish(1)

    def actuate(self, motorValue, motorDuration = 250):
        # need to implement custom message with duration
        self._pub_control_vol.publish(motorValue * 12.6)

    def flipMotor(self):
        self._pub_flip.publish(1)

    def shutdown(self):
        self._pub_shutdown.publish(1)

    def resetPosition(self):
        self._pub_reset_pos.publish(1)

    def resetTorque(self):
        self._pub_reset_trq.publish(1)

    def setMode(self, mode):
        self._pub_mode.publish(mode)

    def updatePosPID(self, p, i, d):
        pid = Float32MultiArray()
        pid.data = [p, i, d]
        self._pub_pid_pos_set.publish(pid)
        self.addCommand("_pub_pid_pos_set", pid)

    def updateVelPID(self, p, i, d):
        pid = Float32MultiArray()
        pid.data = [p, i, d]
        self._pub_pid_vel_set.publish(pid)
        self.addCommand("_pub_pid_vel_set", pid)

    def updateTrqPID(self, p, i, d):
        pid = Float32MultiArray()
        pid.data = [p, i, d]
        self._pub_pid_trq_set.publish(pid)
        self.addCommand("_pub_pid_trq_set", pid)

    # NOT IMPLEMENTED
    def updateFirmware(self, file_path):
    	packet = tr3_network.Packet()
    	packet.address = self._id
    	packet.cmd = CMD_UPDATE_FIRMWARE_BEGIN
        self._tr3._msgs.add(packet)
        self._tr3.step()
        self._tr3.sleep(2)

        bytes_read = 0
        file_size = os.path.getsize(file_path)

        with open(file_path) as f:
            numPacksSent = 0
            while 1:
                byte_s = f.read(4096)
                if not byte_s:
                    break

                packet = tr3_network.Packet()
                packet.address = self._id
                packet.cmd = CMD_UPDATE_FIRMWARE

                for b in byte_s:
                    bytes_read = bytes_read + 1
                    i = int(b.encode('hex'), 16)
                    print(str(i) + ' - ' + str(round(float(bytes_read) / float(file_size) * 100.0,2)) + '% - ' + str(bytes_read) + '/' + str(file_size))
                    packet.addParam(i)

                msgId = self._tr3._msgs._msgId
                self._tr3._msgs.add(packet)

                while 1:
                    s = self.state()
                    print(s)
                    pid = -2
                    try:
                        pid = int(s)
                    except:
                        print("Error parsing state. Is the device connected? Retrying...")
                        self._tr3.sleep(0.500)
                        return

                    if pid == numPacksSent + 1:
                        numPacksSent = numPacksSent + 1
                        break
                    self._tr3.step()

        self._tr3.sleep(2)

        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_UPDATE_FIRMWARE_END
        self._tr3._msgs.add(packet)
        self._tr3.step()
        self._tr3.sleep(2)
