#!/usr/bin/env python

import time
import sys
import math
import tf
import rospy

from tr3py.tr3 import TR3
from tr3py import tr3_utils
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tr3_msgs.msg import ActuatorState

class TR3_Node:
    tr3 = None

    tr3_odom_pub = None
    tr3_state_pub = None
    tr3_state_a0_pub = None
    tr3_state_a1_pub = None
    tr3_state_a2_pub = None
    tr3_state_a3_pub = None
    tr3_state_a4_pub = None
    tr3_state_g0_pub = None
    tr3_state_h0_pub = None
    tr3_state_h1_pub = None
    tr3_state_b0_pub = None
    tr3_state_b1_pub = None

    append_states = False

    def __init__(self, tr3 = None, init_node = True):
        if tr3 == None:
            self.tr3 = TR3()
        else:
            self.tr3 = tr3

        if init_node == True:
            rospy.init_node('tr3_node', anonymous=True)

        rospy.Subscriber("/tr3/shutdown", Bool, self.shutdown_tr3)
        rospy.Subscriber("/tr3/powerup", Bool, self.powerup_tr3)
        rospy.Subscriber("/tr3/mode", UInt8, self.mode_tr3)
        rospy.Subscriber("/tr3/mode/servo", Bool, self.mode_tr3_servo)
        rospy.Subscriber("/tr3/mode/backdrive", Bool, self.mode_tr3_backdrive)
        rospy.Subscriber("/tr3/mode/rotate", Bool, self.mode_tr3_rotate)
        rospy.Subscriber("/tr3/base/diff/cmd_vel", Twist, self.base_cmd)
        rospy.Subscriber("/tr3/joints/a0/mode", UInt8, self.mode_a0)
        rospy.Subscriber("/tr3/joints/a1/mode", UInt8, self.mode_a1)
        rospy.Subscriber("/tr3/joints/a2/mode", UInt8, self.mode_a2)
        rospy.Subscriber("/tr3/joints/a3/mode", UInt8, self.mode_a3)
        rospy.Subscriber("/tr3/joints/a4/mode", UInt8, self.mode_a4)
        rospy.Subscriber("/tr3/joints/g0/mode", UInt8, self.mode_g0)
        rospy.Subscriber("/tr3/joints/h0/mode", UInt8, self.mode_h0)
        rospy.Subscriber("/tr3/joints/h1/mode", UInt8, self.mode_h1)
        rospy.Subscriber("/tr3/joints/b0/mode", UInt8, self.mode_b0)
        rospy.Subscriber("/tr3/joints/b1/mode", UInt8, self.mode_b1)
        rospy.Subscriber("/tr3/joints/a0/reset", Bool, self.reset_a0)
        rospy.Subscriber("/tr3/joints/a1/reset", Bool, self.reset_a1)
        rospy.Subscriber("/tr3/joints/a2/reset", Bool, self.reset_a2)
        rospy.Subscriber("/tr3/joints/a3/reset", Bool, self.reset_a3)
        rospy.Subscriber("/tr3/joints/a4/reset", Bool, self.reset_a4)
        rospy.Subscriber("/tr3/joints/g0/reset", Bool, self.reset_g0)
        rospy.Subscriber("/tr3/joints/h0/reset", Bool, self.reset_h0)
        rospy.Subscriber("/tr3/joints/h1/reset", Bool, self.reset_h1)
        rospy.Subscriber("/tr3/joints/b0/reset", Bool, self.reset_b0)
        rospy.Subscriber("/tr3/joints/b1/reset", Bool, self.reset_b1)
        rospy.Subscriber("/tr3/joints/a0/flip", Bool, self.flip_a0)
        rospy.Subscriber("/tr3/joints/a1/flip", Bool, self.flip_a1)
        rospy.Subscriber("/tr3/joints/a2/flip", Bool, self.flip_a2)
        rospy.Subscriber("/tr3/joints/a3/flip", Bool, self.flip_a3)
        rospy.Subscriber("/tr3/joints/a4/flip", Bool, self.flip_a4)
        rospy.Subscriber("/tr3/joints/g0/flip", Bool, self.flip_g0)
        rospy.Subscriber("/tr3/joints/h0/flip", Bool, self.flip_h0)
        rospy.Subscriber("/tr3/joints/h1/flip", Bool, self.flip_h1)
        rospy.Subscriber("/tr3/joints/b0/flip", Bool, self.flip_b0)
        rospy.Subscriber("/tr3/joints/b1/flip", Bool, self.flip_b1)
        rospy.Subscriber("/tr3/joints/a0/pid", Float32MultiArray, self.pid_a0)
        rospy.Subscriber("/tr3/joints/a1/pid", Float32MultiArray, self.pid_a1)
        rospy.Subscriber("/tr3/joints/a2/pid", Float32MultiArray, self.pid_a2)
        rospy.Subscriber("/tr3/joints/a3/pid", Float32MultiArray, self.pid_a3)
        rospy.Subscriber("/tr3/joints/a4/pid", Float32MultiArray, self.pid_a4)
        rospy.Subscriber("/tr3/joints/h0/pid", Float32MultiArray, self.pid_h0)
        rospy.Subscriber("/tr3/joints/h1/pid", Float32MultiArray, self.pid_h1)
        rospy.Subscriber("/tr3/joints/b0/pid", Float32MultiArray, self.pid_b0)
        rospy.Subscriber("/tr3/joints/b1/pid", Float32MultiArray, self.pid_b1)
        rospy.Subscriber("/tr3/stop", Bool, self.tr3_stop)
        rospy.Subscriber("/tr3/joints/a0/stop", Bool, self.tr3_a0_stop)
        rospy.Subscriber("/tr3/joints/a1/stop", Bool, self.tr3_a1_stop)
        rospy.Subscriber("/tr3/joints/a2/stop", Bool, self.tr3_a2_stop)
        rospy.Subscriber("/tr3/joints/a3/stop", Bool, self.tr3_a3_stop)
        rospy.Subscriber("/tr3/joints/a4/stop", Bool, self.tr3_a4_stop)
        rospy.Subscriber("/tr3/joints/g0/stop", Bool, self.tr3_g0_stop)
        rospy.Subscriber("/tr3/joints/h0/stop", Bool, self.tr3_h0_stop)
        rospy.Subscriber("/tr3/joints/h1/stop", Bool, self.tr3_h1_stop)
        rospy.Subscriber("/tr3/joints/b0/stop", Bool, self.tr3_b0_stop)
        rospy.Subscriber("/tr3/joints/b1/stop", Bool, self.tr3_b1_stop)
        rospy.Subscriber("/tr3/joints/a0/control/position", Float64, self.ctrl_pos_a0)
        rospy.Subscriber("/tr3/joints/a1/control/position", Float64, self.ctrl_pos_a1)
        rospy.Subscriber("/tr3/joints/a2/control/position", Float64, self.ctrl_pos_a2)
        rospy.Subscriber("/tr3/joints/a3/control/position", Float64, self.ctrl_pos_a3)
        rospy.Subscriber("/tr3/joints/a4/control/position", Float64, self.ctrl_pos_a4)
        rospy.Subscriber("/tr3/joints/g0/control/position", Float64, self.ctrl_pos_g0)
        rospy.Subscriber("/tr3/joints/h0/control/position", Float64, self.ctrl_pos_h0)
        rospy.Subscriber("/tr3/joints/h1/control/position", Float64, self.ctrl_pos_h1)
        rospy.Subscriber("/tr3/joints/b0/control/position", Float64, self.ctrl_pos_b0)
        rospy.Subscriber("/tr3/joints/b1/control/position", Float64, self.ctrl_pos_b1)
        rospy.Subscriber("/tr3/joints/a0/control/effort", Float64, self.ctrl_effort_a0)
        rospy.Subscriber("/tr3/joints/a1/control/effort", Float64, self.ctrl_effort_a1)
        rospy.Subscriber("/tr3/joints/a2/control/effort", Float64, self.ctrl_effort_a2)
        rospy.Subscriber("/tr3/joints/a3/control/effort", Float64, self.ctrl_effort_a3)
        rospy.Subscriber("/tr3/joints/a4/control/effort", Float64, self.ctrl_effort_a4)
        rospy.Subscriber("/tr3/joints/h0/control/effort", Float64, self.ctrl_effort_h0)
        rospy.Subscriber("/tr3/joints/h1/control/effort", Float64, self.ctrl_effort_h1)
        rospy.Subscriber("/tr3/joints/b0/control/effort", Float64, self.ctrl_effort_b0)
        rospy.Subscriber("/tr3/joints/b1/control/effort", Float64, self.ctrl_effort_b1)

        self.tr3_odom_pub = rospy.Publisher("/tr3/base/odom", Odometry, queue_size=10)

        self.tr3_state_pub = rospy.Publisher("/tr3/state", JointState, queue_size=1)
        self.tr3_state_a0_pub = rospy.Publisher("/tr3/joints/a0/state", ActuatorState, queue_size=1)
        self.tr3_state_a1_pub = rospy.Publisher("/tr3/joints/a1/state", ActuatorState, queue_size=1)
        self.tr3_state_a2_pub = rospy.Publisher("/tr3/joints/a2/state", ActuatorState, queue_size=1)
        self.tr3_state_a3_pub = rospy.Publisher("/tr3/joints/a3/state", ActuatorState, queue_size=1)
        self.tr3_state_a4_pub = rospy.Publisher("/tr3/joints/a4/state", ActuatorState, queue_size=1)
        self.tr3_state_g0_pub = rospy.Publisher("/tr3/joints/g0/state", ActuatorState, queue_size=1)
        self.tr3_state_h0_pub = rospy.Publisher("/tr3/joints/h0/state", ActuatorState, queue_size=1)
        self.tr3_state_h1_pub = rospy.Publisher("/tr3/joints/h1/state", ActuatorState, queue_size=1)
        self.tr3_state_b0_pub = rospy.Publisher("/tr3/joints/b0/state", ActuatorState, queue_size=1)
        self.tr3_state_b1_pub = rospy.Publisher("/tr3/joints/b1/state", ActuatorState, queue_size=1)

        self.tr3.state_change = self.tr3_state_change

    def shutdown_tr3(self, d):
        b = bool(d)
        if b == True:
            self.tr3.shutdown()

    def powerup_tr3(self, d):
        b = bool(d)
        if b == True:
            self.tr3.powerup()

    def change_mode(self, d, j = -1):
        b = int(d)
        mode = TR3.mode_rotate
        if b == 1:
            mode = TR3.mode_backdrive
        elif b == 2:
            mode = TR3.mode_servo
        elif b == 3:
            mode = TR3.mode_velocity

        if j == -1:
            self.tr3.setMode(mode)
        else:
            j.setMode(mode)

    def stop(self, d, j = -1):
        b = bool(d)
        if b == True:
            if j == -1:
                self.tr3.stop()
            else:
                j.stop()
        else:
            if j == -1:
                self.tr3.release()
            else:
                j.release()

    def ctrl_pos(self, d, j):
        j.setPosition(d)

    def ctrl_effort(self, d, j):
        j.actuate(d)

    def mode_tr3(self, msg):
        self.change_mode(msg.data)

    def mode_tr3_servo(self, msg):
        b = bool(msg.data)
        if b == True:
            self.tr3.setMode(TR3.mode_servo)
        else:
            self.tr3.setMode(TR3.mode_rotate)

    def mode_tr3_backdrive(self, msg):
        b = bool(msg.data)
        if b == True:
            self.tr3.setMode(TR3.mode_backdrive)
        else:
            self.tr3.setMode(TR3.mode_rotate)

    def mode_tr3_rotate(self, msg):
        b = bool(msg.data)
        if b == True:
            self.tr3.setMode(TR3.mode_rotate)
        else:
            self.tr3.setMode(TR3.mode_rotate)

    def base_cmd(self, msg):
        x, th = (msg.linear.x, msg.angular.z)

        R = 0.3175
        L = 0.6562
        l = ((2 * x) - (th * L)) / (2 * R)
        r = ((2 * x) + (th * L)) / (2 * R)

        self.tr3.b0.setVelocity(-l)
        self.tr3.b1.setVelocity(r)

    def mode_a0(self, msg):
        self.change_mode(msg.data, self.tr3.a0)

    def mode_a1(self, msg):
        self.change_mode(msg.data, self.tr3.a1)

    def mode_a2(self, msg):
        self.change_mode(msg.data, self.tr3.a2)

    def mode_a3(self, msg):
        self.change_mode(msg.data, self.tr3.a3)

    def mode_a4(self, msg):
        self.change_mode(msg.data, self.tr3.a4)

    def mode_g0(self, msg):
        self.change_mode(msg.data, self.tr3.g0)

    def mode_h0(self, msg):
        self.change_mode(msg.data, self.tr3.h0)

    def mode_h1(self, msg):
        self.change_mode(msg.data, self.tr3.h1)

    def mode_b0(self, msg):
        self.change_mode(msg.data, self.tr3.b0)

    def mode_b1(self, msg):
        self.change_mode(msg.data, self.tr3.b1)

    def reset(self, b, a):
        if b == True:
            a.resetEncoderPosition()

    def reset_a0(self, msg):
        self.reset(bool(msg.data), self.tr3.a0)

    def reset_a1(self, msg):
        self.reset(bool(msg.data), self.tr3.a1)

    def reset_a2(self, msg):
        self.reset(bool(msg.data), self.tr3.a2)

    def reset_a3(self, msg):
        self.reset(bool(msg.data), self.tr3.a3)

    def reset_a4(self, msg):
        self.reset(bool(msg.data), self.tr3.a4)

    def reset_g0(self, msg):
        self.reset(bool(msg.data), self.tr3.g0)

    def reset_h0(self, msg):
        self.reset(bool(msg.data), self.tr3.h0)

    def reset_h1(self, msg):
        self.reset(bool(msg.data), self.tr3.h1)

    def reset_b0(self, msg):
        self.reset(bool(msg.data), self.tr3.b0)

    def reset_b1(self, msg):
        self.reset(bool(msg.data), self.tr3.b1)

    def pid(self, d, a):
        a.updatePID(d[0], d[1], d[2])

    def pid_a0(self, msg):
        self.pid(msg.data, self.tr3.a0)

    def pid_a1(self, msg):
        self.pid(msg.data, self.tr3.a1)

    def pid_a2(self, msg):
        self.pid(msg.data, self.tr3.a2)

    def pid_a3(self, msg):
        self.pid(msg.data, self.tr3.a3)

    def pid_a4(self, msg):
        self.pid(msg.data, self.tr3.a4)

    def pid_h0(self, msg):
        self.pid(msg.data, self.tr3.h0)

    def pid_h1(self, msg):
        self.pid(msg.data, self.tr3.h1)

    def pid_b0(self, msg):
        self.pid(msg.data, self.tr3.b0)

    def pid_b1(self, msg):
        self.pid(msg.data, self.tr3.b1)

    def flip(self, b, a):
        if b == True:
            a.flipMotor()

    def flip_a0(self, msg):
        self.flip(bool(msg.data), self.tr3.a0)

    def flip_a1(self, msg):
        self.flip(bool(msg.data), self.tr3.a1)

    def flip_a2(self, msg):
        self.flip(bool(msg.data), self.tr3.a2)

    def flip_a3(self, msg):
        self.flip(bool(msg.data), self.tr3.a3)

    def flip_a4(self, msg):
        self.flip(bool(msg.data), self.tr3.a4)

    def flip_g0(self, msg):
        self.flip(bool(msg.data), self.tr3.g0)

    def flip_h0(self, msg):
        self.flip(bool(msg.data), self.tr3.h0)

    def flip_h1(self, msg):
        self.flip(bool(msg.data), self.tr3.h1)

    def flip_b0(self, msg):
        self.flip(bool(msg.data), self.tr3.b0)

    def flip_b1(self, msg):
        self.flip(bool(msg.data), self.tr3.b1)

    def tr3_stop(self, msg):
        self.stop(msg.data)

    def tr3_a0_stop(self, msg):
        self.stop(msg.data, self.tr3.a0)

    def tr3_a1_stop(self, msg):
        self.stop(msg.data, self.tr3.a1)

    def tr3_a2_stop(self, msg):
        self.stop(msg.data, self.tr3.a2)

    def tr3_a3_stop(self, msg):
        self.stop(msg.data, self.tr3.a3)

    def tr3_a4_stop(self, msg):
        self.stop(msg.data, self.tr3.a4)

    def tr3_g0_stop(self, msg):
        self.stop(msg.data, self.tr3.g0)

    def tr3_h0_stop(self, msg):
        self.stop(msg.data, self.tr3.h0)

    def tr3_h1_stop(self, msg):
        self.stop(msg.data, self.tr3.h1)

    def tr3_b0_stop(self, msg):
        self.stop(msg.data, self.tr3.b0)

    def tr3_b1_stop(self, msg):
        self.stop(msg.data, self.tr3.b1)

    def ctrl_pos_a0(self, msg):
        self.ctrl_pos(msg.data, self.tr3.a0)

    def ctrl_pos_a1(self, msg):
        self.ctrl_pos(msg.data, self.tr3.a1)

    def ctrl_pos_a2(self, msg):
        self.ctrl_pos(msg.data, self.tr3.a2)

    def ctrl_pos_a3(self, msg):
        self.ctrl_pos(msg.data, self.tr3.a3)

    def ctrl_pos_a4(self, msg):
        self.ctrl_pos(msg.data, self.tr3.a4)

    def ctrl_pos_g0(self, msg):
        self.ctrl_pos(msg.data, self.tr3.g0)

    def ctrl_pos_h0(self, msg):
        self.ctrl_pos(msg.data, self.tr3.h0)

    def ctrl_pos_h1(self, msg):
        self.ctrl_pos(msg.data, self.tr3.h1)

    def ctrl_pos_b0(self, msg):
        self.ctrl_pos(msg.data, self.tr3.b0)

    def ctrl_pos_b1(self, msg):
        self.ctrl_pos(msg.data, self.tr3.b1)

    def ctrl_effort_a0(self, msg):
        self.ctrl_effort(msg.data, self.tr3.a0)

    def ctrl_effort_a1(self, msg):
        self.ctrl_effort(msg.data, self.tr3.a1)

    def ctrl_effort_a2(self, msg):
        self.ctrl_effort(msg.data, self.tr3.a2)

    def ctrl_effort_a3(self, msg):
        self.ctrl_effort(msg.data, self.tr3.a3)

    def ctrl_effort_a4(self, msg):
        self.ctrl_effort(msg.data, self.tr3.a4)

    def ctrl_effort_h0(self, msg):
        self.ctrl_effort(msg.data, self.tr3.h0)

    def ctrl_effort_h1(self, msg):
        self.ctrl_effort(msg.data, self.tr3.h1)

    def ctrl_effort_b0(self, msg):
        self.ctrl_effort(msg.data, self.tr3.b0)

    def ctrl_effort_b1(self, msg):
        self.ctrl_effort(msg.data, self.tr3.b1)

    def tr3_state_change(self, state):
        if self.append_states == False:
            state[0].append('g0_b')
            state[1].append(0.0)
            state[2].append(0.0)
            state[3].append(0.0)
            state[4].append(0.0)
            append_states = True

        joint_state = JointState()
        joint_state.name = state[0]
        joint_state.position = state[1]
        joint_state.velocity = state[3]
        joint_state.effort = state[2]

        g0_pos = 0.0
        for i in range(len(joint_state.name)):
            if joint_state.name[i] == 'g0':
                joint_state.position[i] = joint_state.position[i] * 0.041
                g0_pos = joint_state.position[i]

        for i in range(len(joint_state.name)):
            if joint_state.name[i] == 'g0_b':
                joint_state.position[i] = -g0_pos

        self.tr3_state_pub.publish(joint_state)

        for i in range(len(state[0])):
            try:
                pub = getattr(self, "tr3_state_" + id + "_pub")
                actuator_state = ActuatorState()
                actuator_state.id = state[0][i]
                actuator_state.position = state[1][i]
                actuator_state.rotations = state[2][i]
                actuator_state.effort = state[3][i]
                actuator_state.velocity = state[4][i]
                actuator_state.mode = state[5][i]
                actuator_state.stop = state[6][i]
                actuator_state.temperature = state[7][i]
                pub.publish(s)
            except:
                pass

	odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.tr3.pos_th)

	odom = Odometry()
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = "odom"

	# set the position
	odom.pose.pose = Pose(Point(self.tr3.pos_x, self.tr3.pos_y, 0), Quaternion(*odom_quat))

	# set the velocity
	odom.child_frame_id = "base_link"
	odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        # publish the message
	br = tf.TransformBroadcaster()
	br.sendTransform((self.tr3.pos_x, self.tr3.pos_y, 0), odom_quat, rospy.Time.now(), "base_link", "odom")
	#br = tf.TransformBroadcaster()
	#br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "map", "odom")
	self.tr3_odom_pub.publish(odom)

    def step(self):
        self.tr3.step()

    def spin(self):
        self.tr3.spin()

if __name__ == '__main__':
    tr3_node = TR3_Node()
    tr3_node.spin()
