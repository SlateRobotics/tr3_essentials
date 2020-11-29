#!/usr/bin/env python

import sys
import time
import signal
import math
import os
import io
import yaml
from datetime import datetime

import tf
import rospy

from tr3_joint import Joint

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import UInt32
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tr3_msgs.msg import ActuatorState
from tr3_msgs.msg import ActuatorPositionCommand

TAU = math.pi * 2.0

close = False
def signal_handler(sig, frame):
    global close
    close = True
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class TR3:
    _state = None
    _pub_odom = None
    _pub_state = None
    _pub_poweron = None
    _pub_poweroff = None
    _pub_power_ota_start = None
    _pub_power_ota_data = None
    _pub_power_ota_end = None

    joints = ["b0","b1","a0","a1","a2","a3","a4","g0","h0","h1"]

    l_pos_prev = None
    r_pos_prev = None
    pos_x = 0
    pos_y = 0
    pos_th = 0

    MODE_STOP = 0
    MODE_SERVO = 1
    MODE_VELOCITY = 2
    MODE_TORQUE = 3
    MODE_ROTATE = 4
    MODE_BACKDRIVE = 5
    MODE_CALIBRATE = 6
    MODE_UPDATE_FIRMWARE = 7

    def __init__(self, _init_node = True):
        self.b0 = Joint(self, "b0")
        self.b1 = Joint(self, "b1")
        self.a0 = Joint(self, "a0")
        self.a1 = Joint(self, "a1")
        self.a2 = Joint(self, "a2")
        self.a3 = Joint(self, "a3")
        self.a4 = Joint(self, "a4")
        self.g0 = Joint(self, "g0")
        self.h0 = Joint(self, "h0")
        self.h1 = Joint(self, "h1")
        #self.p0 = Joint(self, "p0")
        #self.p1 = Joint(self, "p1")
        #self.p2 = Joint(self, "p2")

        rospy.Subscriber("/tr3/shutdown", Bool, self._sub_shutdown)
        rospy.Subscriber("/tr3/powerup", Bool, self._sub_powerup)
        rospy.Subscriber("/tr3/stop", Bool, self._sub_stop)
        rospy.Subscriber("/tr3/home", Bool, self._sub_home)
        rospy.Subscriber("/tr3/base/diff/cmd_vel", Twist, self._sub_base_cmd)
        rospy.Subscriber("/tr3/power/state", Bool, self._sub_power_state)
        rospy.Subscriber("/tr3/power/firmware/update", String, self._sub_power_firmware_update)

        self._pub_poweron = rospy.Publisher("/tr3/power/on", Bool, queue_size=1)
        self._pub_poweroff = rospy.Publisher("/tr3/power/off", Bool, queue_size=1)
        self._pub_odom = rospy.Publisher("/tr3/base/odom", Odometry, queue_size=10)
        self._pub_joint_states = rospy.Publisher("/tr3/joint_states", JointState, queue_size=1)
        self._pub_power_ota_start = rospy.Publisher("/tr3/power/ota/start", UInt32, queue_size=1)
        self._pub_power_ota_data = rospy.Publisher("/tr3/power/ota/data", UInt8MultiArray, queue_size=1)
        self._pub_power_ota_end = rospy.Publisher("/tr3/power/ota/end", Bool, queue_size=1)

        if _init_node == True:
            rospy.init_node('tr3_node', anonymous=True)

        self.powerup()
        print("TR3 ready")

    def state(self):
        return self._state

    def _sub_power_state (self, msg):
        self.powerOn = msg.data
    
    def _sub_powerup (self, msg):
        if msg.data == True:
            self.powerup()
    
    def _sub_shutdown (self, msg):
        if msg.data == True:
            self.shutdown()

    def _sub_stop (self, msg):
        if msg.data == True:
            self.stop()
        else:
            self.release()

    def _sub_home (self, msg):
        if msg.data == True:
            self.a0.setPosition(0.0, 10000)
            self.a1.setPosition(0.0, 10000)
            self.a2.setPosition(0.0, 10000)
            self.a3.setPosition(0.0, 10000)
            self.a4.setPosition(0.0, 10000)
            self.g0.setPosition(1.0, 10000)

    def _sub_base_cmd (self, msg):
        x, th = (msg.linear.x, msg.angular.z)
        self.drive(x, th)

    def _sub_power_firmware_update (self, msg):
        self.updateFirmware("power", msg.data);

    def drive(self, x, th):
        R = 0.3175
        L = 0.6562
        l = ((2 * x) - (th * L)) / (2 * R)
        r = ((2 * x) + (th * L)) / (2 * R)

        self.tr3.b0.setVelocity(-l)
        self.tr3.b1.setVelocity(r)

    def release(self):
        for j in self.joints:
            getattr(self, j).release()

    def powerup(self):
        self._pub_poweron.publish(True)

    def shutdown(self):
        for j in self.joints:
            getattr(self, j).shutdown()
                
        self.sleep(2)
        self.close()

    def stop(self):
        for j in self.joints:
            getattr(self, j).stop()

    def sleep(self, sec):
        t_start = time.time()
        while time.time() - t_start < sec:
            self.step()

    def getJoint(self, id):
        try:
            return getattr(self,id)
        except:
            pass

    def set_state(self):
        joint_state = JointState()
        joint_state.name = []
        joint_state.position = []
        joint_state.velocity = []
        joint_state.effort = []

        for j in self.joints:
            s = getattr(self, j).state()
            if s != None:
                joint_state.name.append(j)
                joint_state.position.append(s.position)
                joint_state.velocity.append(s.velocity)
                joint_state.effort.append(s.torque)

        self._pub_joint_states.publish(joint_state)

    def step(self):
        self.set_state()
        self.step_odom()

    def step_odom(self):
        wheel_dist = 0.6562

        if (self.b0.state() == None or self.b1.state() == None):
            return

        l_pos = -self.b0.state().position
        r_pos = self.b1.state().position

        if (self.l_pos_prev == None or self.r_pos_prev == None):
            self.l_pos_prev = l_pos
            self.r_pos_prev = r_pos
            return
        else:
            d_l = l_pos - self.l_pos_prev
            d_r = r_pos - self.r_pos_prev

            self.l_pos_prev = l_pos
            self.r_pos_prev = r_pos

        dist_per_rad = 0.15875 # meters
        dist_l = d_l * dist_per_rad
        dist_r = d_r * dist_per_rad
        dist_c = (dist_l + dist_r) / 2.0

        self.pos_x += dist_c * math.cos(self.pos_th)
        self.pos_y += dist_c * math.sin(self.pos_th)
        self.pos_th += (dist_r - dist_l) / wheel_dist

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
        self._pub_odom.publish(odom)

    def spin(self, condition = True):
        global close
        r = rospy.Rate(20)
        while condition == True and close == False and rospy.is_shutdown() == False:
            self.step()
            r.sleep()

    def close(self):
        self._pub_poweroff.publish(True)
        self.sleep(1)

    def updateFirmware(self, node_id, file_path):
        pub_start = None
        pub_data = None
        pub_end = None

        if node_id == "power":
            pub_start = self._pub_power_ota_start
            pub_data = self._pub_power_ota_data
            pub_end = self._pub_power_ota_end
        else:
            pub_start = getattr(self, node_id)._pub_ota_start
            pub_data = getattr(self, node_id)._pub_ota_data
            pub_end = getattr(self, node_id)._pub_ota_end

        bytes_read = 0
        file_size = os.path.getsize(file_path)

        pub_start.publish(UInt32(file_size))
        self.sleep(1.0)

        with open(file_path) as f:
            numPacksSent = 0
            while 1:
                bytes_to_read = 512 #4096
                byte_s = f.read(bytes_to_read)
                if not byte_s:
                    break

                ma = UInt8MultiArray()
                ma.data = []

                for b in byte_s:
                    bytes_read = bytes_read + 1
                    byte_int = int(b.encode('hex'), 16)
                    ma.data.append(byte_int)

                pub_ota_data.publish(ma)
                self.sleep(0.050)

        pub_ota_end.publish(0)
        self.sleep(2)

if __name__ == '__main__':
    tr3 = TR3()
    tr3.spin()
