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
from tr3_msgs.msg import ActuatorPositionCommand

class TR3_Node:
    tr3 = None

    tr3_odom_pub = None
    tr3_state_pub = None

    append_states = False

    def __init__(self, tr3 = None, init_node = True):
        if tr3 == None:
            self.tr3 = TR3()
        else:
            self.tr3 = tr3

        if init_node == True:
            rospy.init_node('tr3_node', anonymous=True)

        rospy.Subscriber("/tr3/shutdown", Bool, self.shutdown)
        rospy.Subscriber("/tr3/powerup", Bool, self.powerup)
        rospy.Subscriber("/tr3/mode", UInt8, self.mode_tr3)
        rospy.Subscriber("/tr3/mode/servo", Bool, self.mode_tr3_servo)
        rospy.Subscriber("/tr3/mode/backdrive", Bool, self.mode_tr3_backdrive)
        rospy.Subscriber("/tr3/mode/rotate", Bool, self.mode_tr3_rotate)
        rospy.Subscriber("/tr3/base/diff/cmd_vel", Twist, self.base_cmd)
        rospy.Subscriber("/tr3/stop", Bool, self.tr3_stop)

        for j in ["b0","b1","a0","a1","a2","a3","a4","g0","h0","h1"]:
            rospy.Subscriber("/tr3/joints/" + j + "/mode", UInt8, getattr(self, "mode_" + j))
            rospy.Subscriber("/tr3/joints/" + j + "/reset", Bool, getattr(self, "reset_" + j))
            rospy.Subscriber("/tr3/joints/" + j + "/flip", Bool, getattr(self, "flip_" + j))
            rospy.Subscriber("/tr3/joints/" + j + "/pid/set", Float32MultiArray, getattr(self, "pid_" + j))
            rospy.Subscriber("/tr3/joints/" + j + "/stop", Bool, getattr(self, "stop_" + j))
            exec("rospy.Subscriber(\"/tr3/joints/" + j + "/control/position\", ActuatorPositionCommand, self.pos_" + j + ")")
            exec("rospy.Subscriber(\"/tr3/joints/" + j + "/control/velocity\", Float64, self.vel_" + j + ")")
            rospy.Subscriber("/tr3/joints/" + j + "/control/effort", Float64, getattr(self, "effort_" + j))

            setattr(self, "tr3_state_" + j + "_pub", rospy.Publisher("/tr3/joints/" + j + "/state", ActuatorState, queue_size=1))
            setattr(self, "tr3_pid_pos_" + j + "_pub", rospy.Publisher("/tr3/joints/" + j + "/pid/pos", Float32MultiArray, queue_size=1))
            setattr(self, "tr3_pid_vel_" + j + "_pub", rospy.Publisher("/tr3/joints/" + j + "/pid/vel", Float32MultiArray, queue_size=1))
            setattr(self, "tr3_pid_trq_" + j + "_pub", rospy.Publisher("/tr3/joints/" + j + "/pid/trq", Float32MultiArray, queue_size=1))

        self.tr3_odom_pub = rospy.Publisher("/tr3/base/odom", Odometry, queue_size=10)
        self.tr3_state_pub = rospy.Publisher("/tr3/state", JointState, queue_size=1)
        self.tr3.state_change = self.tr3_state_change

        rospy.Timer(rospy.Duration(0.1), self.publish_pid)

    def shutdown(self, d):
        if bool(d) == True:
            self.tr3.shutdown()

    def powerup(self, d):
        if bool(d) == True:
            self.tr3.powerup()

    def mode(self, d, j = -1):
        mode = TR3.mode_rotate
        if int(d) == 1:
            mode = TR3.mode_backdrive
        elif int(d) == 2:
            mode = TR3.mode_servo
        elif int(d) == 3:
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

    def vel(self, d, j):
        j.setVelocity(d)

    def pos(self, d, j):
        j.setPosition(d.position, d.duration)

    def effort(self, d, j):
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

    def reset(self, b, a):
        if b == True:
            a.resetEncoderPosition()

    def pid(self, d, a):
        a.updatePID(d[0], d[1], d[2])

    def flip(self, b, a):
        if b == True:
            a.flipMotor()

    def tr3_stop(self, msg):
        self.stop(msg.data)

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
            pub_name = "tr3_state_" + state[0][i] + "_pub"
            if hasattr(self, pub_name):
                pub = getattr(self, pub_name)
                actuator_state = ActuatorState()
                actuator_state.id = state[0][i]
                actuator_state.position = state[1][i]
                actuator_state.rotations = int(state[2][i])
                actuator_state.effort = state[3][i]
                actuator_state.velocity = state[4][i]
                actuator_state.torque = state[5][i]
                actuator_state.mode = int(state[6][i])
                actuator_state.stop = bool(state[7][i])
                actuator_state.temperature = state[8][i]
                pub.publish(actuator_state)

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

    def publish_pid(self, event):
        m = Float32MultiArray()
        for j in self.tr3.joints:
            m.data = getattr(self.tr3, j)._pid_pos
            getattr(self, "tr3_pid_pos_" + j + "_pub").publish(m)
            m.data = getattr(self.tr3, j)._pid_vel
            getattr(self, "tr3_pid_vel_" + j + "_pub").publish(m)
            m.data = getattr(self.tr3, j)._pid_trq
            getattr(self, "tr3_pid_trq_" + j + "_pub").publish(m)

    def spin(self):
        self.tr3.spin()


for j in ["b0","b1","a0","a1","a2","a3","a4","g0","h0","h1"]:
    exec("def mode_" + j + "(self, msg): self.mode(msg.data, self.tr3." + j + ")")
    exec("def reset_" + j + "(self, msg): self.reset(bool(msg.data), self.tr3." + j + ")")
    exec("def pid_" + j + "(self, msg): self.pid(msg.data, self.tr3." + j + ")")
    exec("def flip_" + j + "(self, msg): self.flip(bool(msg.data), self.tr3." + j + ")")
    exec("def stop_" + j + "(self, msg): self.stop(msg.data, self.tr3." + j + ")")
    exec("def pos_" + j + "(self, msg): self.pos(msg, self.tr3." + j + ")")
    exec("def vel_" + j + "(self, msg): self.vel(msg.data, self.tr3." + j + ")")
    exec("def effort_" + j + "(self, msg): self.effort(msg.data, self.tr3." + j + ")")

    exec("setattr(TR3_Node, \"mode_" + j +"\", mode_" + j + ")")
    exec("setattr(TR3_Node, \"reset_" + j + "\", reset_" + j + ")")
    exec("setattr(TR3_Node, \"pid_" + j + "\", pid_" + j +")")
    exec("setattr(TR3_Node, \"flip_" + j + "\", flip_" + j + ")")
    exec("setattr(TR3_Node, \"stop_" + j + "\", stop_" + j + ")")
    exec("setattr(TR3_Node, \"pos_" + j + "\", pos_" + j + ")")
    exec("setattr(TR3_Node, \"vel_" + j + "\", vel_" + j + ")")
    exec("setattr(TR3_Node, \"effort_" + j + "\", effort_" + j + ")")

if __name__ == '__main__':
    tr3_node = TR3_Node()
    tr3_node.spin()
