#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <std_msgs/Float64.h>
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
#include "Dynamics.h"
#include "LinearRegression.h"

#define ACS_N 64

class Controller {
  private:
    int mode = MODE_ROTATE;
    int modePrev = MODE_ROTATE;
    bool calibrate = false;

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

    int acs_readings[ACS_N];
    float acs_offset = 2048;

    PID pidPos = PID(&pidPosInput, &pidPosOutput, &pidPosSetpoint);
    PID pidVel = PID(&pidVelInput, &pidVelOutput, &pidVelSetpoint);
    PID pidTrq = PID(&pidTrqInput, &pidTrqOutput, &pidTrqSetpoint);

    LinearRegression regression;

    float SEA_SPRING_RATE = -340.0; // Newton-Meters per Radian
    float SEA_COEFF_M = -504.51;
    float SEA_COEFF_B = -6.4775;

    ControllerState state;
    Encoder encoderTorque = Encoder(64);
    Encoder encoderOutput = Encoder(65);
    LED led;
    Motor motor = Motor(PIN_MTR_PWM, PIN_MTR_IN1, PIN_MTR_IN2);
    MPU9250 imu = MPU9250(Wire,0x68);
    Trajectory trajectory = Trajectory(&state);
    Timer imuTimer = Timer(5); // hz
    Timer logTimer = Timer(1);
    Timer limitTimer = Timer(4);
    Timer calibrateTimer = Timer(5);
    Timer dynamicsTimer = Timer(4);

    void computeState();

  public:
    bool requireImu = true;
    double a1_pos = 0.0;
    double a2_pos = 0.0;
    double a3_pos = 0.0;
    bool a1_pos_recv = false;
    bool a2_pos_recv = false;
    bool a3_pos_recv = false;
    double expected_torque = 0.0;
    double expected_torque_min = MIN_TORQUE;
    double expected_torque_max = MAX_TORQUE;
    bool flag_send_commands = false;

    Storage storage;

    Controller () { }

    void setUp();
    void setUpImu();
    void setUpConfig();

    void setActuatorState(tr3_msgs::ActuatorState* state);
    void setActuatorStatePos(std_msgs::Float64* state_pos);
    void setLimits(std_msgs::Float32MultiArray* limits);
    void setPidPosTunings(std_msgs::Float32MultiArray* gains);
    void setPidVelTunings(std_msgs::Float32MultiArray* gains);
    void setPidTrqTunings(std_msgs::Float32MultiArray* gains);

    void updateExpectedTorque();

    void fanOn();
    void fanOff();

    void step();
    void step_imu();
    void step_rotate();
    void step_servo(bool pulseLED = true);
    void step_velocity(bool pulseLED = true);
    void step_torque(bool pulseLED = true);
    void step_motor();
    void step_calibrate();
    void step_stop();
    void step_temperature();

    void cmd_setMode(uint8_t m);
    void cmd_setPosition(float position, int duration);
    void cmd_setVelocity(float velocity);
    void cmd_setTorque(float torque);
    void cmd_setVoltage(float voltage);
    void cmd_reset();
    void cmd_resetPosition();
    void cmd_resetTorque();
    void cmd_flipMotorPins();
    void cmd_release();
    void cmd_stop();
    void cmd_calibrateStart();
    void cmd_calibrateEnd();
    void cmd_shutdown();
    void cmd_setSpringRate(float v);
    void cmd_setLimitPositionMin(float v);
    void cmd_setLimitPositionMax(float v);
    void cmd_setLimitVelocityMin(float v);
    void cmd_setLimitVelocityMax(float v);
    void cmd_setLimitTorqueMin(float v);
    void cmd_setLimitTorqueMax(float v);
    void cmd_updatePidPos(int i, float v);
    void cmd_updatePidVel(int i, float v);
    void cmd_updatePidTrq(int i, float v);
};

#endif
