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

    /*if (imuTimer.ready()) {
        step_imu();
    }*/
}

void Controller::setActuatorState (tr3_msgs::ActuatorState* msg) {
    msg->position = state.position;
    msg->velocity = state.velocity;
    msg->effort = state.effort;
    msg->torque = state.torque;
}

void Controller::setPidPosTunings (std_msgs::Float32MultiArray* msg) {
    msg->data_length = 3;
    msg->data = (float*)malloc(sizeof(float) * 3);
    msg->data[0] = (float)pidPos.GetTunings(0);
    msg->data[1] = (float)pidPos.GetTunings(1);
    msg->data[2] = (float)pidPos.GetTunings(2);
}

void Controller::setPidVelTunings (std_msgs::Float32MultiArray* msg) {
    msg->data_length = 3;
    msg->data = (float*)malloc(sizeof(float) * 3);
    msg->data[0] = (float)pidVel.GetTunings(0);
    msg->data[1] = (float)pidVel.GetTunings(1);
    msg->data[2] = (float)pidVel.GetTunings(2);
}

void Controller::setPidTrqTunings (std_msgs::Float32MultiArray* msg) {
    msg->data_length = 3;
    msg->data = (float*)malloc(sizeof(float) * 3);
    msg->data[0] = (float)pidTrq.GetTunings(0);
    msg->data[1] = (float)pidTrq.GetTunings(1);
    msg->data[2] = (float)pidTrq.GetTunings(2);
}

void Controller::fanOn () {
    digitalWrite(PIN_FAN, LOW);
}

void Controller::fanOff () {
    digitalWrite(PIN_FAN, HIGH);
}