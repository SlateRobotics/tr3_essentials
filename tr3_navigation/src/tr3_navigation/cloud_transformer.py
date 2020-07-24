#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math
import datetime
import ros_numpy
import numpy as np
import tf2_py as tf2
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

def quaternionMultiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

class cloud_transformer:
    tf_buffer = None
    tf_listener = None
    tf_transform = None

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def setTf(self):
    	lookup_time = rospy.Time(0)
    	target_frame = "base_link"
    	source_frame = "camera_link_optical"

    	try:
    		self.tf_transform = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time, rospy.Duration(2.0))
    	except tf2.LookupException as ex:
    		rospy.logwarn(str(lookup_time.to_sec()))
    		rospy.logwarn(ex)
    	except tf2.ExtrapolationException as ex:
    		rospy.logwarn(str(lookup_time.to_sec()))
    		rospy.logwarn(ex)


    def scale(self, a, shape):
    	out = np.empty(shape).astype(np.ndarray)
    	for row in range(shape[0]):
    		for col in range(shape[1]):
    			r1 = row * (a.shape[0] / shape[0])
    			c1 = col * (a.shape[1] / shape[1])
    			out[row][col] = a[r1][c1]
    	return out

    def transform(self, cloud):
        self.setTf()
    	cloud_in = ros_numpy.numpify(cloud).astype(np.ndarray)
    	cloud_in = self.scale(cloud_in, (48, 64))
    	cloud_out = np.empty((cloud_in.shape[0], cloud_in.shape[1])).astype(np.ndarray)

    	tran = self.tf_transform.transform.translation
    	rot = self.tf_transform.transform.rotation

    	r1 = [rot.w, rot.x, rot.y, rot.z]
    	r2 = [rot.w, -rot.x, -rot.y, -rot.z]
    	t1 = [tran.x, tran.y, tran.z]

    	for row in range(cloud_in.shape[0]):
    		for col in range(cloud_in.shape[1]):
    			p =  cloud_in[row][col]
    			p1 = [0, p[0], p[1], p[2]]

    			if p[0] == None or p[1] == None or p[2] == None:
    				cloud_out[row][col] = [None, None, None, 0]
    			else:
    				p2 = quaternionMultiply(quaternionMultiply(r1, p1), r2)
    				p2 = [p2[1], p2[2], p2[3]]
    				p2 = np.add(p2, t1)
    				cloud_out[row][col] = [p2[0], p2[1], p2[2], 0]

    	return cloud_out
