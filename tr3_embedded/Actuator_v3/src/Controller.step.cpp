#include "Controller.h"
#include "Trajectory.h"

void Controller::step () {
    led.step();
    encoderTorque.step();
    encoderOutput.step();
    computeState();

    Serial.println(expected_torque, 4);

    fanOn();

    switch (mode) {
        case MODE_UPDATE_FIRMWARE:
            motor.stop();
            break;
        case MODE_STOP:
            step_stop();
            break;
        case MODE_ROTATE:
            step_rotate();
            break;
        case MODE_BACKDRIVE:
            pidTrqSetpoint = 0;
            step_torque();
            break;
        case MODE_TORQUE:
            step_torque();
            break;
        case MODE_SERVO:
            step_servo();
            break;
        case MODE_VELOCITY:
            step_velocity();
            break;
        case MODE_CALIBRATE:
            step_calibrate();
            break;
        default:
            step_stop();
            break;
    }

    //if (logTimer.ready()) {
    if (!trajectory.complete()) {
        Serial.print(millis());
        Serial.print("::");
        Serial.print(pidPosSetpoint);
        Serial.print(", ");
        Serial.print(state.position);
        Serial.print(", ");
        Serial.print(pidPosOutput);
        Serial.print("::");
        Serial.print(pidVelSetpoint);
        Serial.print(", ");
        Serial.print(state.velocity);
        Serial.print(", ");
        Serial.print(pidVelOutput);
        Serial.print("::");
        Serial.print(pidTrqSetpoint);
        Serial.print(", ");
        Serial.print(state.torque);
        Serial.print(", ");
        Serial.print(pidTrqOutput);
        Serial.print(", ");
        Serial.println(pidPwrSetpoint);
    }
}

void Controller::step_imu () {
    imu.readSensor();

    /*state.imu_accel_length = 3;
    //state.imu_accel = { 0, 0, 0 };
    state.imu_accel[0] = imu.getAccelX_mss();
    state.imu_accel[1] = imu.getAccelY_mss();
    state.imu_accel[2] = imu.getAccelZ_mss();*/


    /*state.gyro[0] = imu.getGyroX_rads();
    state.gyro[1] = imu.getGyroY_rads();
    state.gyro[2] = imu.getGyroZ_rads();
    state.mag[0] = imu.getMagX_uT();
    
    state.mag[1] = imu.getMagY_uT();
    state.mag[2] = imu.getMagZ_uT();
    state.temp = imu.getTemperature_C();*/
}

void Controller::step_rotate () {
    led.pulse(LED_CYAN);
    motor.executePreparedCommand();
}

void Controller::step_servo () {
    led.pulse(LED_MAGENTA);

    if (NODE_ID == "g0") {
        motor.executePreparedCommand();
        return;
    }

    trajectory.step();

    pidPosInput = state.position;
    pidPosSetpoint = trajectory.getTargetPosition();
    pidPosSetpoint = constrain(pidPosSetpoint, MIN_POSITION, MAX_POSITION);
    pidPos.Compute();

    if (!trajectory.complete()) {
        pidVelInput = state.velocity;
        pidVelSetpoint = trajectory.getTargetVelocity();
        step_velocity(false);
    } else {
        pidVelOutput = 0;
        pidVel.clear();
    }

    MIN_TORQUE = expected_torque - 7.5;
    MAX_TORQUE = expected_torque + 7.5;
    pidTrqSetpoint = pidPosOutput + pidTrqInput;
    step_torque(false);
}

void Controller::step_velocity (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_GREEN);
    }

    if (NODE_ID == "g0") {
        return;
    }

    pidVelInput = state.velocity;
    pidVelSetpoint = constrain(pidVelSetpoint, MIN_VELOCITY, MAX_VELOCITY);
    pidVel.Compute();

    // disable vel controller and wind-down if torque outside bounds
    if (state.torque <= MIN_TORQUE || state.torque >= MAX_TORQUE) {
        pidVelOutput = 0;
        pidVel.clear();
    }
}

void Controller::step_torque (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_YELLOW);
    }

    if (NODE_ID == "g0") {
        return;
    }

    pidTrqInput = state.torque;
    pidTrqSetpoint = constrain(pidTrqSetpoint, MIN_TORQUE, MAX_TORQUE);
    pidTrq.Compute();

    pidPwrSetpoint = pidVelOutput + pidTrqOutput;
    pidPwrSetpoint = constrain(pidPwrSetpoint, DEFAULT_MOTOR_MIN, DEFAULT_MOTOR_MAX);

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