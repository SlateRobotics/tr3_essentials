#!/usr/bin/env python

import time
import sys
import math
import datetime
import actionlib
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult

tr3_a0_state = None
tr3_a1_state = None
tr3_a2_state = None
tr3_a3_state = None
tr3_a4_state = None
tr3_a0_pub = None
tr3_a1_pub = None
tr3_a2_pub = None
tr3_a3_pub = None
tr3_a4_pub = None
tr3_mode_servo_pub = None
tr3_stop_pub = None
tr3_arm_action_server = None

def cb_tr3_a0_state(msg):
	global tr3_a0_state
	tr3_a0_state = float(msg.data)

def cb_tr3_a1_state(msg):
	global tr3_a1_state
	tr3_a1_state = float(msg.data)

def cb_tr3_a2_state(msg):
	global tr3_a2_state
	tr3_a2_state = float(msg.data)

def cb_tr3_a3_state(msg):
	global tr3_a3_state
	tr3_a3_state = float(msg.data)

def cb_tr3_a4_state(msg):
	global tr3_a4_state
	tr3_a4_state = float(msg.data)

def tr3_arm_follow_joint_trajectory(goal):
	global tr3_arm_action_server, tr3_a0_state, tr3_a1_state, tr3_a2_state, tr3_a3_state, tr3_a4_state, tr3_a0_pub, tr3_a1_pub, tr3_a2_pub, tr3_a3_pub, tr3_a4_pub, tr3_mode_servo_pub, tr3_stop_pub

	tr3_mode_servo_pub.publish(1)
	#tr3_stop_pub.publish(0)

	time.sleep(0.200)

	success = True
	feedback = FollowJointTrajectoryFeedback()
	result = FollowJointTrajectoryResult()

	joint_names = goal.trajectory.joint_names
	feedback.joint_names = joint_names

	t_start = datetime.datetime.now()
	for point in goal.trajectory.points:
                print point
		tr3_a0_pub.publish(point.positions[0])
		tr3_a1_pub.publish(point.positions[1])
		tr3_a2_pub.publish(point.positions[2])
		tr3_a3_pub.publish(point.positions[3])
		tr3_a4_pub.publish(point.positions[4])

		'''if tr3_arm_action_server.is_preempt_requested():
			tr3_arm_action_server.set_preempted()
			success = False
			break'''

		while (datetime.datetime.now() - t_start).total_seconds() < point.time_from_start.nsecs / 1000000000.0:
			feedback.desired = point
			feedback.actual.positions = [tr3_a0_state, tr3_a1_state, tr3_a2_state, tr3_a3_state, tr3_a4_state]
			#feedback.error.positions
			tr3_arm_action_server.publish_feedback(feedback)

	tr3_arm_action_server.set_succeeded(result)
	#tr3_stop_pub.publish(1)

def program():
	global tr3_arm_action_server, tr3_arm_follow_joint_trajectory, tr3_a0_state, tr3_a1_state, tr3_a2_state, tr3_a3_state, tr3_a4_state, tr3_a0_pub, tr3_a1_pub, tr3_a2_pub, tr3_a3_pub, tr3_a4_pub, tr3_mode_servo_pub, tr3_stop_pub

	rospy.init_node('tr3_action_server')

	rospy.Subscriber("/tr3/joints/a0/state", Float64, cb_tr3_a0_state)
	rospy.Subscriber("/tr3/joints/a1/state", Float64, cb_tr3_a1_state)
	rospy.Subscriber("/tr3/joints/a2/state", Float64, cb_tr3_a2_state)
	rospy.Subscriber("/tr3/joints/a3/state", Float64, cb_tr3_a3_state)
	rospy.Subscriber("/tr3/joints/a4/state", Float64, cb_tr3_a4_state)
	tr3_a0_pub = rospy.Publisher("/tr3/joints/a0/control/position", Float64, queue_size=1)
	tr3_a1_pub = rospy.Publisher("/tr3/joints/a1/control/position", Float64, queue_size=1)
	tr3_a2_pub = rospy.Publisher("/tr3/joints/a2/control/position", Float64, queue_size=1)
	tr3_a3_pub = rospy.Publisher("/tr3/joints/a3/control/position", Float64, queue_size=1)
	tr3_a4_pub = rospy.Publisher("/tr3/joints/a4/control/position", Float64, queue_size=1)
	tr3_mode_servo_pub = rospy.Publisher("/tr3/mode/servo", Bool, queue_size=1)
	tr3_stop_pub = rospy.Publisher("/tr3/stop", Bool, queue_size=1)

	tr3_arm_action_server = actionlib.SimpleActionServer('/tr3/arm/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=tr3_arm_follow_joint_trajectory, auto_start=False)
	tr3_arm_action_server.start()
	rospy.spin()

if __name__ == '__main__':
	program()
