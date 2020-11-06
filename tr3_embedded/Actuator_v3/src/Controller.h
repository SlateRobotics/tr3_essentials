#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESPmDNS.h>
#include <Update.h>

#include "Config.h"
#include "Utils.h"
#include "ControllerState.h"
#include "Encoder.h"
#include "LED.h"
#include "Motor.h"
#include "MPU9250.h"
#include "NetworkPacket.h"
#include "PID.h"
#include "Timer.h"
#include "Storage.h"

class Controller {
  private:
    int mode = MODE_ROTATE;
    int modePrev = MODE_ROTATE;

    double pidPosInput = 0.0;
    double pidPosSetpoint = 0.0;
    double pidVelInput = 0.0;
    double pidVelSetpoint = 0.0;
    double pidTrqInput = 0.0;
    double pidTrqSetpoint = 0.0;
    double pidPwrSetpoint = 0.0;

    PID pidPos = PID(&pidPosInput, &pidVelSetpoint, &pidPosSetpoint, 8.500, 0.000, 0.000, DIRECT);
    PID pidVel = PID(&pidVelInput, &pidTrqSetpoint, &pidVelSetpoint, 4.000, 4.000, 0.000, DIRECT);
    PID pidTrq = PID(&pidTrqInput, &pidPwrSetpoint, &pidTrqSetpoint, 0.300, 0.050, 0.000, DIRECT);

    long velTrajectoryStart = 0;
    long velTrajectoryDuration = 0;
    double velTrajectoryPosStart = 0;
    const static int velTrajectorySize = 128;
    float velTrajectory[velTrajectorySize];

    float SEA_SPRING_RATE = 75.0; // Newton-Meters per Radian

    ControllerState state;
    Encoder encoderTorque = Encoder(PIN_ENC_TRQ_CS, PIN_ENC_TRQ_CLK, PIN_ENC_TRQ_DO);
    Encoder encoderOutput = Encoder(PIN_ENC_OUT_CS, PIN_ENC_OUT_CLK, PIN_ENC_OUT_DO);
    LED led;
    Motor motor = Motor(PIN_MTR_PWM, PIN_MTR_IN1, PIN_MTR_IN2);
    Storage storage;
    MPU9250 imu = MPU9250(Wire,0x68);
    Timer imuTimer = Timer(5); // hz
    Timer logTimer = Timer(4);

    void computeState();
    void planVelTrajectory();

  public:
    bool requireImu = true;

    Controller () { }

    void setUp();
    void setUpImu();
    void setUpConfig();

    ControllerState* getState();

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

    void parseCmd(NetworkPacket packet);
    void cmd_setMode(NetworkPacket packet);
    void cmd_setPosition(NetworkPacket packet);
    void cmd_setVelocity(NetworkPacket packet);
    void cmd_resetPosition();
    void cmd_flipMotorPins();
    void cmd_rotate(NetworkPacket packet);
    void cmd_release();
    void cmd_stop();
    void cmd_calibrate();
    void cmd_shutdown();
    void cmd_updatePid(NetworkPacket packet);
};

#endif
