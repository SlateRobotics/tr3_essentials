#!/usr/bin/env python

import time
import os
import math
import yaml
import io
import struct
import tr3_network

CMD_UPDATE_FIRMWARE_BEGIN = 0x01
CMD_UPDATE_FIRMWARE = 0x02
CMD_UPDATE_FIRMWARE_END = 0x03
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
CMD_UPDATE_PID_POS = 0x20
CMD_UPDATE_PID_VEL = 0x22
CMD_UPDATE_PID_TRQ = 0x23
CMD_SET_VELOCITY = 0x21

class Joint:
    _tr3 = None
    _id = None
    _position = None
    _rotations = None
    _effort = None
    _torque = None
    _mode = None
    _stop = None
    _temperature = None
    _pid_pos = [9.000, 0.000, 0.000]
    _pid_vel = [4.000, 4.000, 0.000]
    _pid_trq = [0.300, 0.050, 0.000]

    def __init__(self, t, i):
        self._tr3 = t
        self._id = i

    def state(self):
        return self._state

    def setPosition(self, pos, dur = 0):
        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_SET_POS

        for b in bytearray(struct.pack("f", pos)):
            packet.addParam(int(b))

        packet.addParam(int(math.floor(dur % 256)))
        packet.addParam(int(math.floor(dur / 256)))

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def setVelocity (self, vel):
        x = (vel + 10.0) * 100.0

        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_SET_VELOCITY
        packet.addParam(int(math.floor(x % 256)))
        packet.addParam(int(math.floor(x / 256)))

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def release(self):
        cmd = CMD_STOP_RELEASE

        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = cmd

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def stop(self):
        cmd = CMD_STOP_EMERGENCY

        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = cmd

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def actuate(self, motorValue, motorDuration = 250):
        offsetBinary = 128
        x = int(math.floor(motorValue * 100.0))

        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_ROTATE
        packet.addParam(x + offsetBinary)
        packet.addParam(int(math.floor(motorDuration % 256)))
        packet.addParam(int(math.floor(motorDuration / 256)))

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def flipMotor(self):
        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_FLIP_MOTOR

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def shutdown(self):
        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_SHUTDOWN

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def resetEncoderPosition(self):
        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_RESET_POS

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def calibrate(self):
        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_CALIBRATE

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def setMode(self, mode):
        packet = tr3_network.Packet()
        packet.address = self._id
        packet.cmd = CMD_SET_MODE
        packet.addParam(mode)

        self._tr3._msgs.add(packet)
        self._tr3.step()

    def updatePID_pos(self, p = None, i = None, d = None):
        self.updatePID("_pid_pos", p, i, d)

    def updatePID_vel(self, p = None, i = None, d = None):
        self.updatePID("_pid_vel", p, i, d)

    def updatePID_trq(self, p = None, i = None, d = None):
        self.updatePID("_pid_trq", p, i, d)

    def updatePID(self, pid, p = None, i = None, d = None):
        if p == None:
            p = getattr(self, pid)[0]

        if i == None:
            i = getattr(self, pid)[1]

        if d == None:
            d = getattr(self, pid)[2]

        setattr(self, pid, [p, i, d])
        self._tr3.flagSavePID = True

        packet = tr3_network.Packet()
        packet.address = self._id

        if pid == "_pid_pos":
            packet.cmd = CMD_UPDATE_PID_POS
        elif pid == "_pid_vel":
            packet.cmd = CMD_UPDATE_PID_VEL
        elif pid == "_pid_trq":
            packet.cmd = CMD_UPDATE_PID_TRQ

        packet.addParam(int(math.floor(p * 1000.0)))
        packet.addParam(int(math.floor(i * 1000.0)))
        packet.addParam(int(math.floor(d * 1000.0)))

        self._tr3._msgs.add(packet)
        self._tr3.step()


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
