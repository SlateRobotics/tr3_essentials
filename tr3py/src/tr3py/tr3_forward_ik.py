#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tr3_msgs.srv import ForwardIK

pub = None

def handle_joint_states(msg):
    forward_ik = rospy.ServiceProxy('forward_ik', ForwardIK)
    res = forward_ik(msg)
    pub.publish(res.pose)

if __name__ == '__main__':
    rospy.init_node('tr3_forward_ik')
    pub = rospy.Publisher("/tr3/arm/pose", Pose, queue_size=1)
    rospy.Subscriber("/tr3/joint_states", JointState, handle_joint_states)
    rospy.spin()
