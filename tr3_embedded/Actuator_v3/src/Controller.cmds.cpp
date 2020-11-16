#include "Config.h"
#include "Controller.h"

void Controller::cmd_setMode (uint8_t m) {
    mode = m;
    if (mode == MODE_SERVO) {
        pidPosSetpoint = (double)state.position;
    } else if (mode == MODE_VELOCITY) {
        pidVelSetpoint = 0;
    } else if (mode == MODE_TORQUE) {
        pidTrqSetpoint = 0;
    } else if (mode == MODE_BACKDRIVE) {
        pidTrqSetpoint = 0;
    }
}

void Controller::cmd_setPosition (float position, int duration) {
    if (ACTUATOR_ID == "g0") {
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
    } else {
        trajectory.begin(position, duration);
    }

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
    Serial.println(voltage);
    
    motor.prepareCommand(floor(voltage / 12.6 * 100.0), 1000);

    if (mode != MODE_STOP) {
        mode = MODE_ROTATE;
    }
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
    encoderTorque.resetPos();

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
    mode = MODE_ROTATE;
}

void Controller::cmd_stop () {
    if (mode != MODE_STOP) {
        modePrev = mode;
    } else {
        modePrev = MODE_ROTATE;
    }
    mode = MODE_STOP;
}

void Controller::cmd_calibrate() {
    encoderOutput.resetPos();
    encoderTorque.resetPos();

    if (ACTUATOR_ID == "a0" || ACTUATOR_ID == "a1" || ACTUATOR_ID == "a2" || ACTUATOR_ID == "b0" || ACTUATOR_ID == "b1") {
        SEA_SPRING_RATE = -350;
    } else {
        SEA_SPRING_RATE = -350;
    }
    storage.writeFloat(EEADDR_SEA_SPRING_RATE, SEA_SPRING_RATE);

    uint16_t encO_offset = encoderOutput.getOffset();
    uint16_t encD_offset = encoderTorque.getOffset();
    storage.writeUInt16(EEADDR_ENC_OUT_OFFSET, encO_offset);
    storage.writeUInt16(EEADDR_ENC_TRQ_OFFSET, encD_offset);
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
        motor.stop();
    }
}

void Controller::cmd_updatePidPos (float p, float i, float d) {
    pidPos.SetTunings(p, i, d);
    storage.writeFloat(EEADDR_PID_POS_P, p);
    storage.writeFloat(EEADDR_PID_POS_I, i);
    storage.writeFloat(EEADDR_PID_POS_D, d);
    storage.commit();
}

void Controller::cmd_updatePidVel (float p, float i, float d) {
    pidVel.SetTunings(p, i, d);
    storage.writeFloat(EEADDR_PID_VEL_P, p);
    storage.writeFloat(EEADDR_PID_VEL_I, i);
    storage.writeFloat(EEADDR_PID_VEL_D, d);
    storage.commit();
}

void Controller::cmd_updatePidTrq (float p, float i, float d) {
    pidTrq.SetTunings(p, i, d);
    storage.writeFloat(EEADDR_PID_TRQ_P, p);
    storage.writeFloat(EEADDR_PID_TRQ_I, i);
    storage.writeFloat(EEADDR_PID_TRQ_D, d);
    storage.commit();
}