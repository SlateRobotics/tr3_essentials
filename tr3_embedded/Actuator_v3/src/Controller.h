#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <std_msgs/Float32MultiArray.h>
#include <tr3_msgs/ActuatorState.h>

#include <ESPmDNS.h>
#include <Update.h>

#include "Config.h"
#include "Utils.h"
#include "ControllerState.h"
#include "Encoder.h"
#include "LED.h"
#include "Motor.h"
#include "MPU9250.h"
#include "PID.h"
#include "Trajectory.h"
#include "Timer.h"
#include "Storage.h"

class Controller {
  private:
    int mode = MODE_ROTATE;
    int modePrev = MODE_ROTATE;

    float MIN_POSITION = DEFAULT_POSITION_MIN;
    float MAX_POSITION = DEFAULT_POSITION_MAX;
    float MIN_VELOCITY = DEFAULT_VELOCITY_MIN;
    float MAX_VELOCITY = DEFAULT_VELOCITY_MAX;
    float MIN_TORQUE = DEFAULT_TORQUE_MIN;
    float MAX_TORQUE = DEFAULT_TORQUE_MAX;

    double pidPosInput = 0.0;
    double pidPosSetpoint = 0.0;
    double pidPosOutput = 0.0;
    double pidVelInput = 0.0;
    double pidVelSetpoint = 0.0;
    double pidVelOutput = 0.0;
    double pidTrqInput = 0.0;
    double pidTrqSetpoint = 0.0;
    double pidTrqOutput = 0.0;
    double pidPwrSetpoint = 0.0;

    PID pidPos = PID(&pidPosInput, &pidPosOutput, &pidPosSetpoint, 8.500, 0.000, 0.000, DIRECT);
    PID pidVel = PID(&pidVelInput, &pidVelOutput, &pidVelSetpoint, 4.000, 4.000, 0.000, DIRECT);
    PID pidTrq = PID(&pidTrqInput, &pidTrqOutput, &pidTrqSetpoint, 0.250, 0.050, 0.000, DIRECT);

    float SEA_SPRING_RATE = -350.0; // Newton-Meters per Radian

    ControllerState state;
    Encoder encoderTorque = Encoder(PIN_ENC_TRQ_CS, PIN_ENC_TRQ_CLK, PIN_ENC_TRQ_DO);
    Encoder encoderOutput = Encoder(PIN_ENC_OUT_CS, PIN_ENC_OUT_CLK, PIN_ENC_OUT_DO);
    LED led;
    Motor motor = Motor(PIN_MTR_PWM, PIN_MTR_IN1, PIN_MTR_IN2);
    Storage storage;
    MPU9250 imu = MPU9250(Wire,0x68);
    Trajectory trajectory = Trajectory(&state);
    Timer imuTimer = Timer(5); // hz
    Timer logTimer = Timer(4);

    void computeState();

  public:
    bool requireImu = true;

    Controller () { }

    void setUp();
    void setUpImu();
    void setUpConfig();

    void setActuatorState(tr3_msgs::ActuatorState* state);
    void setLimits(std_msgs::Float32MultiArray* limits);
    void setPidPosTunings(std_msgs::Float32MultiArray* gains);
    void setPidVelTunings(std_msgs::Float32MultiArray* gains);
    void setPidTrqTunings(std_msgs::Float32MultiArray* gains);

    void fanOn();
    void fanOff();

    void step();
    void step_imu();
    void step_rotate();
    void step_servo();
    void step_velocity(bool pulseLED = true);
    void step_torque(bool pulseLED = true);
    void step_calibrate();
    void step_stop();

    void cmd_setMode(uint8_t m);
    void cmd_setPosition(float position, int duration);
    void cmd_setVelocity(float velocity);
    void cmd_setTorque(float torque);
    void cmd_setVoltage(float voltage);
    void cmd_resetPosition();
    void cmd_resetTorque();
    void cmd_flipMotorPins();
    void cmd_release();
    void cmd_stop();
    void cmd_calibrate();
    void cmd_shutdown();
    void cmd_setLimitPositionMin(float v);
    void cmd_setLimitPositionMax(float v);
    void cmd_setLimitVelocityMin(float v);
    void cmd_setLimitVelocityMax(float v);
    void cmd_setLimitTorqueMin(float v);
    void cmd_setLimitTorqueMax(float v);
    void cmd_updatePidPos(float p, float i, float d);
    void cmd_updatePidVel(float p, float i, float d);
    void cmd_updatePidTrq(float p, float i, float d);
};

#endif
