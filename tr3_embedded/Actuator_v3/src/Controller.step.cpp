#include "Controller.h"

void Controller::step () {
    encoderTorque.step();
    encoderOutput.step();
    computeState();

    fanOn();

    if (mode == MODE_UPDATE_FIRMWARE) {
        motor.stop();
    } else if (mode == MODE_STOP) {
        step_stop();
    } else if (mode == MODE_ROTATE) {
        step_rotate();
    } else if (mode == MODE_BACKDRIVE) {
        step_torque();
    } else if (mode == MODE_SERVO) {
        step_servo();
    } else if (mode == MODE_VELOCITY) {
        step_velocity();
    } else if (mode == MODE_CALIBRATE) {
        step_calibrate();
    } else {
        step_stop();
    }

    /*Serial.print(millis());
    Serial.print("::");
    Serial.print(state.position);
    Serial.print(", ");
    Serial.print(pidPosSetpoint);
    Serial.print(", ");
    Serial.print(state.velocity);
    Serial.print(", ");
    Serial.print(pidVelSetpoint);
    Serial.print(", ");
    Serial.print(state.torque);
    Serial.print(", ");
    Serial.print(pidTrqSetpoint);
    Serial.print(", ");
    Serial.println(pidPwrSetpoint);*/
}

void Controller::step_imu () {
    imu.readSensor();
    state.accel[0] = imu.getAccelX_mss();
    state.accel[1] = imu.getAccelY_mss();
    state.accel[2] = imu.getAccelZ_mss();
    state.gyro[0] = imu.getGyroX_rads();
    state.gyro[1] = imu.getGyroY_rads();
    state.gyro[2] = imu.getGyroZ_rads();
    state.mag[0] = imu.getMagX_uT();
    
    state.mag[1] = imu.getMagY_uT();
    state.mag[2] = imu.getMagZ_uT();
    state.temp = imu.getTemperature_C();
}

void Controller::step_rotate () {
    led.pulse(LED_CYAN);
    motor.executePreparedCommand();
}

void Controller::step_servo () {
    led.pulse(LED_MAGENTA);

    if (ACTUATOR_ID == "g0") {
        motor.executePreparedCommand();
        return;
    }
    
    pidPosInput = state.position;
    Utils::formatPosition(&pidPosInput, &pidPosSetpoint);

    float acceleration = 0.25; // rad / sec

    long dur_elapsed = millis() - velTrajectoryStart;
    long dur_remain = velTrajectoryDuration - dur_elapsed;

    float dist_total = pidPosSetpoint - velTrajectoryPosStart;
    float dist_remain = pidPosSetpoint - pidPosInput;
    
    float break_dist = (state.velocity * state.velocity) / (acceleration * 2.0);
    
    if (dur_remain < 200 || abs(dist_remain) < break_dist) {
        pidPos.Compute();
    } else {
        float vel_req = dist_remain / (float)dur_remain * 1000.0;
        pidVelSetpoint = vel_req;
    }

    pidVelSetpoint = constrain(pidVelSetpoint, VELOCITY_MIN, VELOCITY_MAX);

    step_velocity(false);
}

void Controller::step_velocity (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_GREEN);
    }

    if (ACTUATOR_ID == "g0") {
        return;
    }

    pidVelInput = state.velocity;
    pidVel.Compute();

    step_torque(false);
}

void Controller::step_torque (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_YELLOW);
    }

    if (ACTUATOR_ID == "g0") {
        return;
    }

    pidTrqInput = state.torque;
    pidTrq.Compute();
    motor.step(pidPwrSetpoint * 100.0);
}

void Controller::step_calibrate() {
    led.blink(0, 255, 0);
    // not implemented
}

void Controller::step_stop () {
    led.pulse(LED_RED);
    pidPos.clear();
    pidVel.clear();
    pidTrq.clear();
    motor.stop();
}