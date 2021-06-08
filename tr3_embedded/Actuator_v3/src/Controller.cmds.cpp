#include "Controller.h"

void Controller::cmd_setMode (uint8_t m) {
    mode = m;
    if (mode == MODE_SERVO) {
        pidPosSetpoint = (double)state.position;
    } else if (mode == MODE_VELOCITY) {
        pidVelSetpoint = 0;
    } else if (mode == MODE_TORQUE) {
        pidTrqSetpoint = expected_torque;
        expected_torque_min = MIN_TORQUE;
        expected_torque_max = MAX_TORQUE;
    } else if (mode == MODE_BACKDRIVE) {
        pidTrqSetpoint = 0;
    }
}

void Controller::cmd_setPosition (float position, int duration) {
    Serial.println(position);
    #if (NODE_ID == NODE_G0)
        if (abs(position) < 0.5 ) {
            if (state.position > 0.5) {
                motor.prepareCommand(100, 1750);
            }
            state.position = 0;
            storage.writeFloat(EEADDR_ENC_OUT_POS, 0.0);
        } else {
            if (state.position < 0.5) {
                motor.prepareCommand(-100, 1750);
            }
            state.position = 1;
            storage.writeFloat(EEADDR_ENC_OUT_POS, 1.0);
        }
    #else
        trajectory.begin(position, duration);
    #endif

    if (mode != MODE_STOP) {
        mode = MODE_SERVO;
    }
}

void Controller::cmd_setVelocity (float velocity) {
    pidVelSetpoint = velocity;

    if (mode != MODE_STOP) {
        mode = MODE_VELOCITY;
    }
}

void Controller::cmd_setTorque (float torque) {
    pidTrqSetpoint = torque;

    if (mode != MODE_STOP) {
        mode = MODE_TORQUE;
    }
}

void Controller::cmd_setVoltage (float voltage) {
    motor.prepareCommand(floor(voltage / 12.6 * 100.0), 200);

    if (mode != MODE_STOP) {
        mode = MODE_ROTATE;
    }
}

void Controller::cmd_reset () {
    storage.reset();
    cmd_resetPosition();
    cmd_resetTorque();
    motor.stop();
    ESP.restart();
}

void Controller::cmd_resetPosition () {
    encoderOutput.resetPos();

    pidPos.clear();
    pidVel.clear();

    uint16_t enc_offset = encoderOutput.getOffset();
    storage.writeUInt16(EEADDR_ENC_OUT_OFFSET, enc_offset);
    storage.commit();
}

void Controller::cmd_resetTorque () {
    encoderTorque.resetPos(expected_torque / SEA_SPRING_RATE);
    if (abs(SEA_COEFF_M) < 0.1) {
        //encoderTorque.resetPos(expected_torque / -350.0);
    } else {
        //encoderTorque.resetPos((expected_torque - SEA_COEFF_B) / SEA_COEFF_M);
    }

    pidPos.clear();
    pidVel.clear();

    uint16_t enc_offset = encoderTorque.getOffset();
    storage.writeUInt16(EEADDR_ENC_TRQ_OFFSET, enc_offset);
    storage.commit();
}

void Controller::cmd_flipMotorPins () {
    motor.flipDrivePins();

    storage.writeBool(EEADDR_MTR_FLIP, motor.flipDrivePinsStatus());
    storage.commit();
}

void Controller::cmd_release () {
    if (mode == MODE_STOP) {
        mode = MODE_ROTATE;
    }
}

void Controller::cmd_stop () {
    if (mode != MODE_STOP) {
        modePrev = mode;
    } else {
        modePrev = MODE_ROTATE;
    }
    mode = MODE_STOP;
}

void Controller::cmd_calibrateStart () {
    calibrate = true;
    encoderTorque.resetPos();
}

void Controller::cmd_calibrateEnd () {
    calibrate = false;

    double values[3];
    regression.getValues(values);
    SEA_COEFF_M = values[0];
    SEA_COEFF_B = values[1];

    Serial.print("M: ");
    Serial.print(SEA_COEFF_M);
    Serial.print(", B: ");
    Serial.println(SEA_COEFF_B);

    storage.writeFloat(EEADDR_SEA_M, SEA_COEFF_M);
    storage.writeFloat(EEADDR_SEA_B, SEA_COEFF_B);
    storage.commit();
}

void Controller::cmd_shutdown() {
    led.off();
    motor.stop();
    computeState();

    storage.writeBool(EEADDR_ENC_OUT_UP, encoderOutput.isUp());
    storage.writeDouble(EEADDR_ENC_OUT_LAP, encoderOutput.getLap());
    storage.writeBool(EEADDR_ENC_TRQ_UP, encoderTorque.isUp());
    storage.writeDouble(EEADDR_ENC_TRQ_LAP, encoderTorque.getLap());
    storage.commit();

    while (1) {
        led.off();
        led.step();
        motor.stop();
    }
}

void Controller::cmd_setSpringRate (float v) {
    SEA_SPRING_RATE = v;
    storage.writeFloat(EEADDR_SEA_SPRING_RATE, v);
    storage.commit();
}

void Controller::cmd_setLimitPositionMin (float v) {
    MIN_POSITION = v;
    storage.writeFloat(EEADDR_POSITION_MIN, MIN_POSITION);
    storage.commit();
}

void Controller::cmd_setLimitPositionMax (float v) {
    MAX_POSITION = v;
    storage.writeFloat(EEADDR_POSITION_MAX, MAX_POSITION);
    storage.commit();
}

void Controller::cmd_setLimitVelocityMin (float v) {
    MIN_VELOCITY = v;
    storage.writeFloat(EEADDR_VELOCITY_MIN, MIN_VELOCITY);
    storage.commit();
}

void Controller::cmd_setLimitVelocityMax (float v) {
    MAX_VELOCITY = v;
    storage.writeFloat(EEADDR_VELOCITY_MAX, MAX_VELOCITY);
    storage.commit();
}

void Controller::cmd_setLimitTorqueMin (float v) {
    MIN_TORQUE = v;
    storage.writeFloat(EEADDR_TORQUE_MIN, MIN_TORQUE);
    storage.commit();
}

void Controller::cmd_setLimitTorqueMax (float v) {
    MAX_TORQUE = v;
    storage.writeFloat(EEADDR_TORQUE_MAX, MAX_TORQUE);
    storage.commit();
}

void Controller::cmd_updatePidPos (int i, float val) {
    pidPos.SetTunings(i, val);

    switch (i) {
        case 0:
            storage.writeFloat(EEADDR_PID_POS_P, val);
            break;
        case 1:
            storage.writeFloat(EEADDR_PID_POS_I, val);
            break;
        case 2:
            storage.writeFloat(EEADDR_PID_POS_D, val);
            break;
        case 3:
            storage.writeFloat(EEADDR_PID_POS_IC, val);
            break;
        case 4:
            storage.writeFloat(EEADDR_PID_POS_FF, val);
            break;
        default:
            break;
    }
    
    storage.commit();
}

void Controller::cmd_updatePidVel (int i, float val) {
    pidVel.SetTunings(i, val);

    switch (i) {
        case 0:
            storage.writeFloat(EEADDR_PID_VEL_P, val);
            break;
        case 1:
            storage.writeFloat(EEADDR_PID_VEL_I, val);
            break;
        case 2:
            storage.writeFloat(EEADDR_PID_VEL_D, val);
            break;
        case 3:
            storage.writeFloat(EEADDR_PID_VEL_IC, val);
            break;
        case 4:
            storage.writeFloat(EEADDR_PID_VEL_FF, val);
            break;
        default:
            break;
    }
    
    storage.commit();
}

void Controller::cmd_updatePidTrq (int i, float val) {
    pidTrq.SetTunings(i, val);

    switch (i) {
        case 0:
            storage.writeFloat(EEADDR_PID_TRQ_P, val);
            break;
        case 1:
            storage.writeFloat(EEADDR_PID_TRQ_I, val);
            break;
        case 2:
            storage.writeFloat(EEADDR_PID_TRQ_D, val);
            break;
        case 3:
            storage.writeFloat(EEADDR_PID_TRQ_IC, val);
            break;
        case 4:
            storage.writeFloat(EEADDR_PID_TRQ_FF, val);
            break;
        default:
            break;
    }
    
    storage.commit();
}