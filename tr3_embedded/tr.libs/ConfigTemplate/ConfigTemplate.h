#ifndef CONFIG_H
#define CONFIG_H

// MATH HELPERS
#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

// ACTUATOR DETAILS
#define ACTUATOR_ID "a2"
#define ACTUATOR_VERSION "v3.0.0"

// ROSSERIAL CONFIG
#define CONFIG_ROSSERIAL_OVER_WIFI 1
#define CONFIG_ROSSERVER_IP "192.168.1.5"
#define CONFIG_ROSSERVER_PORT 11411
#define CONFIG_ROSSERVER_AP "TR_AN_J03NXO"
#define CONFIG_ROSSERVER_PASS "narrowroad512"

// ROSTOPICS USED BY ACTUATOR
#define RT_STATE              "/tr3/joints/" ACTUATOR_ID "/state"
#define RT_MODE               "/tr3/joints/" ACTUATOR_ID "/mode"
#define RT_RESET              "/tr3/joints/" ACTUATOR_ID "/reset"
#define RT_FLIP               "/tr3/joints/" ACTUATOR_ID "/flip"
#define RT_STOP               "/tr3/joints/" ACTUATOR_ID "/stop"
#define RT_SHUTDOWN           "/tr3/joints/" ACTUATOR_ID "/shutdown"
#define RT_CONTROL_POSITION   "/tr3/joints/" ACTUATOR_ID "/control/position"
#define RT_CONTROL_VELOCITY   "/tr3/joints/" ACTUATOR_ID "/control/velocity"
#define RT_CONTROL_TORQUE     "/tr3/joints/" ACTUATOR_ID "/control/torque"
#define RT_CONTROL_VOLTAGE    "/tr3/joints/" ACTUATOR_ID "/control/voltage"
#define RT_PID_POS            "/tr3/joints/" ACTUATOR_ID "/pid_pos"
#define RT_PID_VEL            "/tr3/joints/" ACTUATOR_ID "/pid_vel"
#define RT_PID_TRQ            "/tr3/joints/" ACTUATOR_ID "/pid_trq"
#define RT_PID_POS_SET        "/tr3/joints/" ACTUATOR_ID "/pid_pos/set"
#define RT_PID_VEL_SET        "/tr3/joints/" ACTUATOR_ID "/pid_vel/set"
#define RT_PID_TRQ_SET        "/tr3/joints/" ACTUATOR_ID "/pid_trq/set"

// PINS USED BY ACTUATOR
#define PIN_MTR_PWM 23 // ENABLE / PWM
#define PIN_MTR_IN1 27 // IN1 / DRIVE 1
#define PIN_MTR_IN2 14 // IN2 / DRIVE 2
#define PIN_ENC_TRQ_CS  33
#define PIN_ENC_TRQ_CLK 25
#define PIN_ENC_TRQ_DO  26
#define PIN_ENC_OUT_CS  32
#define PIN_ENC_OUT_CLK 22
#define PIN_ENC_OUT_DO  34
#define PIN_FAN 2
#define LED_DATA_PIN 5

// DEFAULT LIMITS
#define POSITION_MIN -6.28
#define POSITION_MAX 6.28
#define VELOCITY_MIN -0.942
#define VELOCITY_MAX 0.942
#define TORQUE_MIN -60.0
#define TORQUE_MAX 60.0
#define MOTOR_MIN -1.0
#define MOTOR_MAX 1.0

// MODES
#define MODE_STOP 0
#define MODE_SERVO 1
#define MODE_VELOCITY 2
#define MODE_TORQUE 3
#define MODE_ROTATE 4
#define MODE_BACKDRIVE 5
#define MODE_CALIBRATE 6
#define MODE_UPDATE_FIRMWARE 7

#endif
