#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int32MultiArray

import ros_numpy

from tr3_navigation import TR3_Nav

tr3_nav = None
pub = None

processing = False

def signal_handler(sig, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def camera_depth_callback(msg):
    global processing
    if (processing == False):
        processing = True
        time_start = time.clock()

        result = Int32MultiArray()
        result.data = []

        pc2_array = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        #cloud = ros_numpy.point_cloud2.get_xyz_points(pc2_array)#tr3_nav.transform_cloud_msg(msg)
        
        #for row in range(cloud.shape[0]):
        #    for col in range(cloud.shape[1]):
        for point in pc2_array:
            n = 0
            #x = cloud[row][col][0]
            #y = cloud[row][col][1]
            #z = cloud[row][col][2]

            x = point[0]
            y = point[1]
            z = point[2]

            if math.isnan(x):
                n = 1
                x = 0
            else:
                x = int(x * 1000000)

            if math.isnan(y):
                n = 1
                y = 0
            else:
                y = int(y * 1000000)

            if math.isnan(z):
                n = 1
                z = 0
            else:
                z = int(z * 1000000)

            result.data.append(n)
            result.data.append(x)
            result.data.append(y)
            result.data.append(z)

        pub.publish(result)

        time_elapsed = (time.clock() - time_start)
        processing = False

if __name__ == '__main__':
    rospy.init_node("tr3_navigation_node")
    tr3_nav = TR3_Nav()

    rospy.Subscriber("/camera/depth/points", PointCloud2, camera_depth_callback)
    pub = rospy.Publisher("/tr3/depth/scaled", Int32MultiArray, queue_size=10)
    rospy.spin()
