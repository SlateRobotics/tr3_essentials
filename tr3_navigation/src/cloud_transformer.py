#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32MultiArray
from tr3py.tr3_sim import TR3
from tr3_navigation import TR3_Nav

tr3 = TR3()
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

        cloud = tr3_nav.transformer.transform(msg);
        for row in range(cloud.shape[0]):
            for col in range(cloud.shape[1]):
                n = 0
                x = cloud[row][col][0]
                y = cloud[row][col][1]
                z = cloud[row][col][2]

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
    rospy.Subscriber("/camera/depth/points", PointCloud2, camera_depth_callback)
    pub = rospy.Publisher("/tr3/depth/scaled", Int32MultiArray, queue_size=10)
    tr3_nav = TR3_Nav()
    tr3.spin()
