#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math
import tf
from nav_msgs.msg import Odometry

def handle_odom(msg):
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation

    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "map", "odom")

if __name__ == '__main__':
    rospy.init_node('tr3_tf_broadcaster')
    rospy.Subscriber('/tr3/base/odom', Odometry, handle_odom)
    rospy.spin()
