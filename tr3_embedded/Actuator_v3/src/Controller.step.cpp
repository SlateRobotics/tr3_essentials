#include "Controller.h"
#include "Trajectory.h"

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
        pidTrqSetpoint = 0;
        step_torque();
    } else if (mode == MODE_TORQUE) {
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

    /*if (logTimer.ready()) {
        Serial.print(millis());
        Serial.print("::");
        Serial.print(encoderOutput.pos);
        Serial.print(", ");
        Serial.print(encoderOutput.getPrevPosition());
        Serial.print(", ");
        Serial.print(encoderOutput.getOffset());
        Serial.print(", ");
        Serial.print(encoderOutput.getLap());
        Serial.print(", ");
        Serial.print(encoderOutput.getAngleRadians());
        Serial.println();
    }*/

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

    if (ACTUATOR_ID == "g0") {
        motor.executePreparedCommand();
        return;
    }

    trajectory.step();

    pidPosInput = state.position;
    pidPosSetpoint = trajectory.getTargetPosition();
    pidPosSetpoint = constrain(pidPosSetpoint, POSITION_MIN, POSITION_MAX);
    pidPos.Compute();

    pidVelInput = state.velocity;
    pidVelSetpoint = trajectory.getTargetVelocity();
    step_velocity(false);

    // gravity compensation if assigned appropriate torque limits
    // possible
    // pidTrqSetpoint = pidTrqInput;

    pidTrqSetpoint = pidPosOutput + pidTrqInput;
    step_torque(false);
}

void Controller::step_velocity (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_GREEN);
    }

    if (ACTUATOR_ID == "g0") {
        return;
    }

    pidVelInput = state.velocity;
    pidVelSetpoint = constrain(pidVelSetpoint, VELOCITY_MIN, VELOCITY_MAX);
    pidVel.Compute();

    // disable vel controller and wind-down if torque outside bounds
    if (state.torque <= TORQUE_MIN || state.torque >= TORQUE_MAX) {
        pidVelOutput = 0;
        pidVel.clear();
    }
}

void Controller::step_torque (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_YELLOW);
    }

    if (ACTUATOR_ID == "g0") {
        return;
    }

    pidTrqInput = state.torque;
    pidTrqSetpoint = constrain(pidTrqSetpoint, TORQUE_MIN, TORQUE_MAX);
    pidTrq.Compute();

    pidPwrSetpoint = pidVelOutput + pidTrqOutput;
    pidPwrSetpoint = constrain(pidPwrSetpoint, MOTOR_MIN, MOTOR_MAX);
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