#include "Controller.h"

void Controller::computeState () {
    #if (NODE_ID == NODE_G0) 
        state.rotations = 0;
        state.effort = 0;
        state.torque = 0;
        return;
    #else
        if (mode == MODE_STOP) {
            state.stop = true;
        } else {
            state.stop = false;
        }
        
        state.mode = mode;
        state.position = encoderOutput.getAngleRadians();
        state.velocity = encoderOutput.getVelocity();
        state.acceleration = encoderOutput.getAcceleration();
        state.rotations = encoderOutput.getRotations();
        state.effort = motor.getEffort();

        float torque = encoderTorque.getAngleRadians();
        if (torque > PI) { torque -= TAU; }
        state.torque = torque * SEA_SPRING_RATE;

        int reading = analogRead(PIN_TMP36);
        float voltage = reading / 1023.0;
        float temp = (voltage - 0.5) * 100.0; // celcius
        state.temp = (state.temp * 0.99) + (temp * 0.01);
    #endif
}

void Controller::updateExpectedTorque () {
    if (!dynamicsTimer.ready()) return;
    #if (NODE_ID == NODE_A1)
        if (!a2_pos_recv || !a3_pos_recv) return;
        expected_torque = Dynamics::torque_a1(state.position, a2_pos, a3_pos);
    #elif (NODE_ID == NODE_A2)
        if (!a1_pos_recv || !a3_pos_recv) return;
        expected_torque = Dynamics::torque_a2(a1_pos, state.position, a3_pos);
    #elif (NODE_ID == NODE_A3)
        //expected_torque = Dynamics::torque_a3(a1_pos, a2_pos, state.position);
        expected_torque = 0;
    #endif
}

void Controller::setActuatorState (tr3_msgs::ActuatorState* msg) {
    msg->id = NODE_ID_STR;
    msg->mode = state.mode;
    msg->stop = state.stop;
    msg->position = state.position;
    msg->velocity = state.velocity;
    msg->effort = state.effort;
    msg->torque = state.torque;
    msg->temperature = state.temp;
}

void Controller::setActuatorStatePos (std_msgs::Float64* msg) {
    msg->data = state.position;
}

void Controller::setLimits (std_msgs::Float32MultiArray* msg) {
    msg->data_length = 6;
    msg->data = (float*)malloc(sizeof(float) * msg->data_length);
    msg->data[0] = MIN_POSITION;
    msg->data[1] = MAX_POSITION;
    msg->data[2] = MIN_VELOCITY;
    msg->data[3] = MAX_VELOCITY;
    msg->data[4] = MIN_TORQUE;
    msg->data[5] = MAX_TORQUE;
}

void Controller::setPidPosTunings (std_msgs::Float32MultiArray* msg) {
    msg->data_length = 5;
    msg->data = (float*)malloc(sizeof(float) * msg->data_length);
    for (int i = 0; i < msg->data_length; i++) {
        msg->data[i] = (float)pidPos.GetTunings(i);
    }
}

void Controller::setPidVelTunings (std_msgs::Float32MultiArray* msg) {
    msg->data_length = 5;
    msg->data = (float*)malloc(sizeof(float) * msg->data_length);
    for (int i = 0; i < msg->data_length; i++) {
        msg->data[i] = (float)pidVel.GetTunings(i);
    }
}

void Controller::setPidTrqTunings (std_msgs::Float32MultiArray* msg) {
    msg->data_length = 5;
    msg->data = (float*)malloc(sizeof(float) * msg->data_length);
    for (int i = 0; i < msg->data_length; i++) {
        msg->data[i] = (float)pidTrq.GetTunings(i);
    }
}

void Controller::fanOn () {
    digitalWrite(PIN_FAN, HIGH);
}

void Controller::fanOff () {
    digitalWrite(PIN_FAN, LOW);
}