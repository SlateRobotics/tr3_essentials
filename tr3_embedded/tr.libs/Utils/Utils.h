#ifndef TR3_UTILS_H
#define TR3_UTILS_H

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

namespace Utils {
    inline float bytesToFloat (uint8_t a[4]) {
      float value = 0.0;
       byte* p = (byte*)(void*)&value;
       for (int i = 0; i < sizeof(value); i++) {
           *p++ = a[i];
       }
       return value;
    }

    inline double formatAngle (double x) {
        if (x < -TAU) {
            x += TAU;
            return formatAngle(x);
        } else if (x > TAU) {
            x -= TAU;
            return formatAngle(x);
        } else {
            return x;
        }
    }

    inline void formatPosition (double* input, double* setpoint) {
        double d = formatAngle(*input) - formatAngle(*setpoint);
        if (d > PI) {
            *setpoint = formatAngle(*setpoint);
            *setpoint += TAU;
        } else if (d < -PI) {
            *input = formatAngle(*input);
            *input += TAU;
        } else {
            *setpoint = formatAngle(*setpoint);
            *input = formatAngle(*input);
        }
    } 
};

#endif