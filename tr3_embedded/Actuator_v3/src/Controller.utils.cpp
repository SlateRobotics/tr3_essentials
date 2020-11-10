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

void Controller::planVelTrajectory () {
    velTrajectoryPosStart = (double)state.position;
    Utils::formatPosition(&velTrajectoryPosStart, &pidPosSetpoint);

    float v_rat = 1.25; // increase vel to account for controller lag
    
    float posDiff = pidPosSetpoint - velTrajectoryPosStart;
    float v_avg = posDiff / (float)velTrajectoryDuration * 1000.0 * v_rat;
    float t_inc = (float)velTrajectoryDuration / (float)velTrajectorySize / 1000.0;
    
    for (int i = 0; i < velTrajectorySize; i++) {
        velTrajectory[i] = v_avg;
        if ((float)(i + 1) / (float)velTrajectorySize < 0.25) {
            //velTrajectory[i] = (float)(i + 1) * ((1.333 * v_avg) / ((float)velTrajectorySize * 0.25));
        } else if ((float)(i - 1) / (float)velTrajectorySize > 0.75) {
            //velTrajectory[i] = (float)(velTrajectorySize - i - 1) * ((1.333 * v_avg) / ((float)velTrajectorySize * 0.25));
            //velTrajectory[i] = 1.333 * v_avg;
        } else {
            //velTrajectory[i] = 1.333 * v_avg;
        }
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