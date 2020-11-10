#include "Controller.h"

void Controller::parseCmd (NetworkPacket packet) {
    if (packet.command == CMD_UPDATE_FIRMWARE_BEGIN) {
        led.white();
        Serial.println("update");
        mode = MODE_UPDATE_FIRMWARE;
        Update.begin(UPDATE_SIZE_UNKNOWN);
    } else if (packet.command == CMD_UPDATE_FIRMWARE) {
        // doing it this way, we process small chunks of the firmware
        // instead of putting the entire prog into mem, then updating
        led.white();
        Serial.println("recv packet");
        Update.write(packet.parameters, packet.length - 4);
    } else if (packet.command == CMD_UPDATE_FIRMWARE_END) {
        led.white();
        Update.end(true);
        ESP.restart();
    } else if (packet.command == CMD_SET_MODE) {
        cmd_setMode(packet);
    } else if (packet.command == CMD_SET_POS) {
        cmd_setPosition(packet);
    } else if (packet.command == CMD_RESET_POS) {
        cmd_resetPosition();
    } else if (packet.command == CMD_FLIP_MOTOR) {
        cmd_flipMotorPins();
    } else if (packet.command == CMD_ROTATE) {
        cmd_rotate(packet);
    } else if (packet.command == CMD_STOP_RELEASE) {
        cmd_release();
    } else if (packet.command == CMD_STOP_EMERGENCY) {
        cmd_stop();
    } else if (packet.command == CMD_CALIBRATE) {
        cmd_calibrate();
    } else if (packet.command == CMD_SHUTDOWN) {
        cmd_shutdown();
    } else if (packet.command == CMD_UPDATE_PID_POS) {
        cmd_updatePid(packet);
    } else if (packet.command == CMD_UPDATE_PID_VEL) {
        cmd_updatePid(packet);
    } else if (packet.command == CMD_UPDATE_PID_TRQ) {
        cmd_updatePid(packet);
    } else if (packet.command == CMD_SET_VELOCITY) {
        cmd_setVelocity(packet);
    }
}

void Controller::cmd_setMode (NetworkPacket packet) {
    mode = packet.parameters[0];
    if (mode == MODE_SERVO) {
        pidPosSetpoint = (double)state.position;
    } else if (mode == MODE_VELOCITY) {
        pidVelSetpoint = 0;
    } else if (mode == MODE_BACKDRIVE) {
        pidTrqSetpoint = 0;
    }
}

void Controller::cmd_setPosition (NetworkPacket packet) {
    int param = packet.parameters[0] + packet.parameters[1] * 256;
    int dur = packet.parameters[2] + packet.parameters[3] * 256;
    double pos = param / 65535.0 * TAU;

    if (ACTUATOR_ID == "g0") {
        if (abs(pos) < 0.5 ) {
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
        pidPosSetpoint = Utils::formatAngle(pos);
        velTrajectoryDuration = dur;
        planVelTrajectory();
        velTrajectoryStart = millis();
    }

    if (mode != MODE_STOP) {
        mode = MODE_SERVO;
    }
}

void Controller::cmd_setVelocity (NetworkPacket packet) {
    int param = packet.parameters[0] + packet.parameters[1] * 256;
    pidVelSetpoint = (param / 100.0) - 10.0;

    pidVel.clear();

    if (mode != MODE_STOP) {
        mode = MODE_VELOCITY;
    }
}

void Controller::cmd_resetPosition () {
    float positionOutput = encoderOutput.getAngleRadians();
    
    encoderOutput.resetPos();
    encoderTorque.resetPos();

    pidPos.clear();
    pidVel.clear();

    uint16_t encO_offset = encoderOutput.getOffset();
    uint16_t encD_offset = encoderTorque.getOffset();
    storage.writeUInt16(EEADDR_ENC_OUT_OFFSET, encO_offset);
    storage.writeUInt16(EEADDR_ENC_TRQ_OFFSET, encD_offset);
    storage.commit();
}

void Controller::cmd_flipMotorPins () {
    motor.flipDrivePins();

    storage.writeBool(EEADDR_MTR_FLIP, motor.flipDrivePinsStatus());
    storage.commit();
}

void Controller::cmd_rotate (NetworkPacket packet) {
    int offsetBinary = 128;
    int motorStep = packet.parameters[0] - offsetBinary;
    int motorDuration = packet.parameters[1] + packet.parameters[2] * 256;

    if (motorDuration > 1000) {
        motorDuration = 1000;
    }

    motor.prepareCommand(motorStep, motorDuration);
    
    if (mode != MODE_STOP) {
        mode = MODE_ROTATE;
    }
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
        SEA_SPRING_RATE = 75;
    } else {
        SEA_SPRING_RATE = 50;
    }
    storage.writeUInt16(EEADDR_SEA_SPRING_RATE, SEA_SPRING_RATE);

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

void Controller::cmd_updatePid (NetworkPacket packet) {
    double p = packet.parameters[0] / 1000.0;
    double i = packet.parameters[1] / 1000.0;
    double d = packet.parameters[2] / 1000.0;

    if (packet.command == CMD_UPDATE_PID_POS) {
        pidPos.SetTunings(p, i, d);
        storage.writeFloat(EEADDR_PID_POS_P, p);
        storage.writeFloat(EEADDR_PID_POS_I, i);
        storage.writeFloat(EEADDR_PID_POS_D, d);
    } else if (packet.command == CMD_UPDATE_PID_VEL) {
        pidVel.SetTunings(p, i, d);
        storage.writeFloat(EEADDR_PID_VEL_P, p);
        storage.writeFloat(EEADDR_PID_VEL_I, i);
        storage.writeFloat(EEADDR_PID_VEL_D, d);
    } else if (packet.command == CMD_UPDATE_PID_TRQ) {
        pidTrq.SetTunings(p, i, d);
        storage.writeFloat(EEADDR_PID_TRQ_P, p);
        storage.writeFloat(EEADDR_PID_TRQ_I, i);
        storage.writeFloat(EEADDR_PID_TRQ_D, d);
    }

    storage.commit();
}