#ifndef TR3_DYNAMICS_H
#define TR3_DYNAMICS_H

namespace Dynamics {
    const double grav_accel = 9.807;

    const double a1_l1_dist = 0.0950; // meters
    const double a1_a2_dist = 0.1900; // meters
    const double a2_l1_dist = 0.1500; // meters
    const double a2_a3_dist = 0.3000; // meters
    const double a4_g0_dist = 0.1625; // meters, center of mass

    const double l0_mass = 0.500; // kg
    const double a2_mass = 4.085; // kg, + 0.393 (actuator cover)
    const double l1_mass = 1.130; // kg
    const double a3_mass = 2.598; // kg
    const double a4_mass = 3.080; // kg
    const double g0_mass = 2.000; // kg
    

    inline double torque_a1 (double a1_pos, double a2_pos, double a3_pos) {
        // a1 -> l0 : torques from link between a1 and a2
        double a1_l0_x = a1_l1_dist * sin(a1_pos);
        double d_a1_l0 = a1_l0_x;
        double t_a1_l0 = d_a1_l0 * (l0_mass * grav_accel);

        // a1 -> a2 : torques from a2 + L-bracket
        double a1_a2_x = a1_a2_dist * sin(a1_pos);
        double a1_a2_y = a1_a2_dist * cos(a1_pos);
        double d_a1_a2 = a1_a2_x;
        double t_a1_a2 = d_a1_a2 * (a2_mass * grav_accel);

        // a1 -> l1 : torques from link between a2 and a3
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
        double a2_a3_x = a2_a3_dist * cos((PI / 2) - (a2_offset + a2_pos));
        double a2_a3_y = a2_a3_dist * sin((PI / 2) - (a2_offset + a2_pos));
        double a2_a3_x2 = a2_a3_x * cos(a1_pos) - a2_a3_y * sin(a1_pos);
        double a2_a3_y2 = a2_a3_x * sin(a1_pos) + a2_a3_y * cos(a1_pos);
        double a1_a3_x = a2_a3_x2 + a1_a2_x;
        double a1_a3_y = a2_a3_y2 + a1_a2_y;
        double d_a1_a3 = a1_a3_x;
        double t_a1_a3 = d_a1_a3 * (a3_mass * grav_accel);

        // a1 -> a4 : torques from a4 + L-bracket
        double d_a1_a4 = a1_a3_x;
        double t_a1_a4 = d_a1_a4 * (a4_mass * grav_accel);

        // a1 -> g0 : torques from g0
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

        return t_a1_l0 + t_a1_a2 + t_a1_l1 + t_a1_a3 + t_a1_a4 + t_a1_g0;
    }

    inline double torque_a2 (double a1_pos, double a2_pos, double a3_pos) {
        // a1 -> a2 : torques from a2 + L-bracket
        double a1_a2_x = a1_a2_dist * sin(a1_pos);
        double a1_a2_y = a1_a2_dist * cos(a1_pos);
        double d_a1_a2 = a1_a2_x;
        double t_a1_a2 = d_a1_a2 * (a2_mass * grav_accel);

        // a1 -> l1 : torques from link between a2 and a3
        double a2_offset = 0.698132; // rad
        double a2_l1_x = a2_l1_dist * cos((PI / 2) - (a2_offset + a2_pos));
        double a2_l1_y = a2_l1_dist * sin((PI / 2) - (a2_offset + a2_pos));
        double a2_l1_x2 = a2_l1_x * cos(a1_pos) - a2_l1_y * sin(a1_pos);
        double a2_l1_y2 = a2_l1_x * sin(a1_pos) + a2_l1_y * cos(a1_pos);
        a2_l1_x2 += a1_a2_x;
        a2_l1_y2 += a1_a2_y;
        double d_a2_l1 = a2_l1_x2 - d_a1_a2;
        double t_a2_l1 = d_a2_l1 * (l1_mass * grav_accel);

        // a1 -> a3 : torques from a3 + L-bracket
        double a2_a3_x = a2_a3_dist * cos((PI / 2) - (a2_offset + a2_pos));
        double a2_a3_y = a2_a3_dist * sin((PI / 2) - (a2_offset + a2_pos));
        double a2_a3_x2 = a2_a3_x * cos(a1_pos) - a2_a3_y * sin(a1_pos);
        double a2_a3_y2 = a2_a3_x * sin(a1_pos) + a2_a3_y * cos(a1_pos);
        double a1_a3_x = a2_a3_x2 + a1_a2_x;
        double a1_a3_y = a2_a3_y2 + a1_a2_y;
        double d_a2_a3 = a1_a3_x - d_a1_a2;
        double t_a2_a3 = d_a2_a3 * (a3_mass * grav_accel);

        // a1 -> a4 : torques from a4 + L-bracket
        double d_a2_a4 = a1_a3_x - d_a1_a2;
        double t_a2_a4 = d_a2_a4 * (a4_mass * grav_accel);

        // a1 -> g0 : torques from g0
        double a3_offset = 0.698132; // rad
        double a4_g0_x = a4_g0_dist * cos((PI / 2) - (a3_offset + a3_pos));
        double a4_g0_y = a4_g0_dist * sin((PI / 2) - (a3_offset + a3_pos));
        double a4_g0_x2 = a4_g0_x * cos(-a1_pos) - a4_g0_y * sin(-a1_pos);
        double a4_g0_y2 = a4_g0_x * sin(-a1_pos) + a4_g0_y * cos(-a1_pos);
        double a4_g0_x3 = a4_g0_x2 * cos(a2_offset + a2_pos) - a4_g0_y2 * sin(a2_offset + a2_pos);
        double a4_g0_y3 = a4_g0_x2 * sin(a2_offset + a2_pos) + a4_g0_y2 * cos(a2_offset + a2_pos);
        double a1_g0_x = a4_g0_x3 + a1_a3_x;
        double a1_g0_y = a4_g0_y3 + a1_a3_y;
        double d_a2_g0 = a1_g0_x - d_a1_a2;
        double t_a2_g0 = d_a2_g0 * (g0_mass * grav_accel);

        /*Serial.print(a1_pos, 6);
        Serial.print(", ");
        Serial.print(a2_pos, 6);
        Serial.print(", ");
        Serial.print(a3_pos, 6);
        Serial.print(", ");
        Serial.print(d_a1_a2 * 1000.0);
        Serial.print(", ");
        Serial.print(d_a2_l1 * 1000.0);
        Serial.print(", ");
        Serial.print(d_a2_a3 * 1000.0);
        Serial.print(", ");
        Serial.print(d_a2_a4 * 1000.0);
        Serial.print(", ");
        Serial.print(d_a2_g0 * 1000.0);
        Serial.print(", ");
        Serial.println(t_a2_l1 + t_a2_a3 + t_a2_a4 + t_a2_g0);*/

        return t_a2_l1 + t_a2_a3 + t_a2_a4 + t_a2_g0;
    }
}

#endif