#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "Config.h"

namespace Dynamics {
    double torque_a1 (double a1_pos, double a2_pos, double a3_pos) { 
        // given
        double grav_accel = 9.807;

        // a1 -> l0 : torques from link between a1 and a2
        double a1_l1_dist = 0.095; // meters
        double l0_mass = 0.5; // kg
        double a1_l0_x = a1_l1_dist * sin(a1_pos);
        double d_a1_l0 = a1_l0_x;
        double t_a1_l0 = d_a1_l0 * (l0_mass * grav_accel);

        // a1 -> a2 : torques from a2 + L-bracket
        double a1_a2_dist = 0.190; // meters
        double a2_mass = 3.7; // kg
        double a1_a2_x = a1_a2_dist * sin(a1_pos);
        double a1_a2_y = a1_a2_dist * cos(a1_pos);
        double d_a1_a2 = a1_a2_x;
        double t_a1_a2 = d_a1_a2 * (a2_mass * grav_accel);

        // a1 -> l1 : torques from link between a2 and a3
        double a2_l1_dist = 0.150; // meters
        double l1_mass = 0.5; // kg
        double a2_offset = 0.698132; // rad
        double a2_l1_x = a2_l1_dist * cos((PI / 2) - (a2_offset + a2_pos));
        double a2_l1_y = a2_l1_dist * sin((PI / 2) - (a2_offset + a2_pos));
        double a2_l1_x2 = a2_l1_x * cos(a1_pos) - a2_l1_y * sin(a1_pos);
        double a2_l1_y2 = a2_l1_x * sin(a1_pos) + a2_l1_y * cos(a1_pos);
        a2_l1_x2 += a1_a2_x;
        a2_l1_y2 += a1_a2_y;
        double d_a1_l1 = a2_l1_x2;
        double t_a1_l1 = d_a1_l1 * (l1_mass * grav_accel);

        // a1 -> a3 : torques from a3 + L-bracket
        double a2_a3_dist = 0.300; // meters
        double a3_mass = 3.5; // kg
        double a2_a3_x = a2_a3_dist * cos((PI / 2) - (a2_offset + a2_pos));
        double a2_a3_y = a2_a3_dist * sin((PI / 2) - (a2_offset + a2_pos));
        double a2_a3_x2 = a2_a3_x * cos(a1_pos) - a2_a3_y * sin(a1_pos);
        double a2_a3_y2 = a2_a3_x * sin(a1_pos) + a2_a3_y * cos(a1_pos);
        double a1_a3_x = a2_a3_x2 + a1_a2_x;
        double a1_a3_y = a2_a3_y2 + a1_a2_y;
        double d_a1_a3 = a1_a3_x;
        double t_a1_a3 = d_a1_a3 * (a3_mass * grav_accel);

        // a1 -> a4 : torques from a4 + L-bracket
        double a4_mass = 3.5; // kg
        double d_a1_a4 = a1_a3_x;
        double t_a1_a4 = d_a1_a4 * (a4_mass * grav_accel);

        // a1 -> g0 : torques from g0
        double a4_g0_dist = 0.1625; // meters, center of mass
        double g0_mass = 2.0; // kg   
        double a3_offset = 0.698132; // rad
        double a4_g0_x = a4_g0_dist * cos((PI / 2) - (a3_offset + a3_pos));
        double a4_g0_y = a4_g0_dist * sin((PI / 2) - (a3_offset + a3_pos));
        double a4_g0_x2 = a4_g0_x * cos(-a1_pos) - a4_g0_y * sin(-a1_pos);
        double a4_g0_y2 = a4_g0_x * sin(-a1_pos) + a4_g0_y * cos(-a1_pos);
        double a4_g0_x3 = a4_g0_x2 * cos(a2_offset + a2_pos) - a4_g0_y2 * sin(a2_offset + a2_pos);
        double a4_g0_y3 = a4_g0_x2 * sin(a2_offset + a2_pos) + a4_g0_y2 * cos(a2_offset + a2_pos);
        double a1_g0_x = a4_g0_x3 + a1_a3_x;
        double a1_g0_y = a4_g0_y3 + a1_a3_y;
        double d_a1_g0 = a1_g0_x;
        double t_a1_g0 = d_a1_g0 * (g0_mass * grav_accel);

        return t_a1_l0 + t_a1_a2 + t_a1_l1 + t_a1_a3 + t_a1_a4;
    }
}

#endif