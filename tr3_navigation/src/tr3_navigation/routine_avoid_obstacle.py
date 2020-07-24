#!/usr/bin/env python

import sys
import time
import signal
import math

from geometry_msgs.msg import Twist

class avoid_obstacle:
    priority = 2
    flag_set = False

    new_flag = False
    initial_omega = 0.0

    min_dist = 0
    avoid_vectors = []

    def __init__(self):
        pass

    def flag(self, cloud):
        av_coef = 0.01
        self.avoid_vectors = []
        max_dist = 3.0
        for row in range(cloud.shape[0]):
            for col in range(cloud.shape[1]):
                p = cloud[row][col]
                dist = math.sqrt(((0-p[0])**2)+((0-p[1])**2))
                if (dist < max_dist):
                    self.min_dist = dist
                if self.should_avoid(p[0], p[1], p[2], dist):
					x = -1.0 / dist * p[0] * av_coef
					y = -1.0 / dist * p[1] * av_coef
					self.avoid_vectors.append([x, y])

        initial_flag = self.flag_set
        self.flag_set = len(self.avoid_vectors) > 0

        if self.flag_set == True and initial_flag == False:
            self.new_flag = True
            #print "new flag"

        if self.flag_set == True:
            pass
            #print "Objected within", self.min_dist, "meters"

        return self.flag_set

    def step(self):
        avoid_sum = [0, 0]
        for v in self.avoid_vectors:
            avoid_sum[0] = avoid_sum[0] + v[0]
            avoid_sum[1] = avoid_sum[1] + v[1]

        x = avoid_sum[0]
        y = avoid_sum[1]
        #print x, y

        velocity = 0
        if self.min_dist <= 1.0:
            velocity = -2.0
        elif self.min_dist > 1.00 and self.min_dist <= 1.25:
            velocity = 0.0
        elif self.min_dist > 1.25 and self.min_dist <= 1.50:
            velocity = 2.0
        elif self.min_dist > 1.50 and self.min_dist <= 1.75:
            velocity = 4.0
        elif self.min_dist > 1.75 and self.min_dist <= 2.00:
            velocity = 6.0
        elif self.min_dist > 2.00 and self.min_dist <= 3.00:
            velocity = 8.0

        omega = -math.atan(y / x) * 5.0

        if self.new_flag == True:
            self.initial_omega = omega
            self.new_flag = False
        else:
            if (omega > 0 and self.initial_omega < 0):
                omega = -omega
            elif (omega < 0 and self.initial_omega > 0):
                omega = -omega

        cmd_vel = Twist()
        cmd_vel.linear.x = velocity
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = -omega

        return cmd_vel

    def should_avoid(self, x, y, z, dist):
    	dist_max = 3.0
    	z_min = 0.040

    	tr3_x1 = -0.35
    	tr3_x2 = 0.35
    	tr3_y1 = 0.40
    	return ((z > z_min) and (dist < dist_max) and ((x < tr3_x1 or x > tr3_x2) and y > tr3_y1))
