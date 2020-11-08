#ifndef CONFIG_H
#define CONFIG_H

#define ACTUATOR_ID "a2"

#define TR2_AN_SSID "TR2_AN_111222333"
#define TR2_AN_PASS "MATHI78741"

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

#define CMD_UPDATE_FIRMWARE_BEGIN 0x01
#define CMD_UPDATE_FIRMWARE 0x02
#define CMD_UPDATE_FIRMWARE_END 0x03
#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_STOP_RELEASE 0x15
#define CMD_STOP_EMERGENCY 0x16
#define CMD_FLIP_MOTOR 0x17
#define CMD_CALIBRATE 0x18
#define CMD_SHUTDOWN 0x19
#define CMD_UPDATE_PID 0x20
#define CMD_SET_VELOCITY 0x21

#define MODE_UPDATE_FIRMWARE 0x01
#define MODE_CALIBRATE 0x02
#define MODE_STOP 0x0F
#define MODE_SERVO 0x10
#define MODE_BACKDRIVE 0x11
#define MODE_ROTATE 0x12
#define MODE_VELOCITY 0x13

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

#define VELOCITY_MIN -0.942
#define VELOCITY_MAX 0.942

#define TORQUE_MIN -40.0
#define TORQUE_MAX 40.0

#endif
