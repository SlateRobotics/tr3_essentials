#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math
import tf
from geometry_msgs.msg import Twist

pub = None

def handle_cmd(msg):
    cmd_vel = Twist()
    cmd_vel.linear.x = msg.linear.y
    cmd_vel.angular.z = -msg.angular.z
    pub.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('tr3_cmd_map')
    pub = rospy.Publisher("/tr3/base/diff/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, handle_cmd)
    rospy.spin()
