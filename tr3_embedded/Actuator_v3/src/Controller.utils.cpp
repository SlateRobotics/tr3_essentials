#include "Controller.h"

void Controller::computeState () {
    if (ACTUATOR_ID == "g0") {
        state.rotations = 0;
        state.effort = 0;
        state.torque = 0;
        return;
    }
    
    state.position = encoderOutput.getAngleRadians();
    state.velocity = encoderOutput.getVelocity();
    state.acceleration = encoderOutput.getAcceleration();
    state.rotations = encoderOutput.getRotations();
    state.effort = motor.getEffort();

    float torque = encoderTorque.getAngleRadians();
    if (torque > PI) { torque -= TAU; }
    state.torque = torque * SEA_SPRING_RATE;

    if (imuTimer.ready()) {
        step_imu();
    }
}

ControllerState* Controller::getState () {
    return &state;
}

void Controller::fanOn () {
    digitalWrite(PIN_FAN, LOW);
}

void Controller::fanOff () {
    digitalWrite(PIN_FAN, HIGH);
}