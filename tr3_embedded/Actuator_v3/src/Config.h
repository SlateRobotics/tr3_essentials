#ifndef CONFIG_H
#define CONFIG_H

#define ACTUATOR_ID "a0"
#define ACTUATOR_VERSION "v3.0.0"

#define TR2_AN_SSID "TR2_AN_111222333"
#define TR2_AN_PASS "MATHI78741"

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

#define VELOCITY_MIN -0.942
#define VELOCITY_MAX 0.942

#define TORQUE_MIN -40.0
#define TORQUE_MAX 40.0

#endif
