#include "Controller.h"

void Controller::step () {
    led.step();
    encoderTorque.step();
    encoderOutput.step();
    computeState();

    fanOn();

    if (calibrate == true) {
        step_calibrate();
    }

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
            step_motor();
            break;
        case MODE_TORQUE:
            step_torque();
            step_motor();
            break;
        case MODE_SERVO:
            step_servo();
            step_motor();
            break;
        case MODE_VELOCITY:
            step_velocity();
            step_motor();
            break;
        case MODE_CALIBRATE:
            step_calibrate();
            break;
        default:
            step_stop();
            break;
    }

    if (logTimer.ready()) {
    //if (!trajectory.complete()) {
        /*Serial.print(SEA_SPRING_RATE, 8);
        Serial.print(", ");
        Serial.print(state.position, 8);
        Serial.print(", ");
        Serial.print(expected_torque, 8);
        Serial.print(", ");
        Serial.println(encoderTorque.getAngleRadians(), 8);*/
        Serial.print(millis());
        Serial.print("::");
        Serial.print(expected_torque);
        Serial.print(", ");
        Serial.print(expected_torque_min);
        Serial.print(", ");
        Serial.print(expected_torque_max);
        Serial.print("::");
        Serial.print(pidPosSetpoint);
        Serial.print(", ");
        Serial.print(state.position);
        Serial.print(", ");
        Serial.print(pidPos.getIntegralSum());
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
        Serial.print(pidTrq.getIntegralSum());
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

void Controller::step_servo (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_MAGENTA);
    }

    #if (NODE_ID == NODE_G0)
        motor.executePreparedCommand();
        return;
    #else
        trajectory.step();

        pidPosInput = state.position;
        pidPosSetpoint = trajectory.getTargetPosition();
        pidPosSetpoint = constrain(pidPosSetpoint, MIN_POSITION, MAX_POSITION);

        // compute limits for torque based on set position
        if (limitTimer.ready()) {
            #if (NODE_ID == NODE_A1)
                if (a2_pos_recv && a3_pos_recv) {
                    double et = Dynamics::torque_a1(pidPosSetpoint, a2_pos, a3_pos);
                    expected_torque_min = et - 10.0;
                    expected_torque_max = et + 10.0;
                }
            #elif (NODE_ID == NODE_A2)
                if (a1_pos_recv && a3_pos_recv) {
                    double et = Dynamics::torque_a2(a1_pos, pidPosSetpoint, a3_pos);
                    expected_torque_min = et - 10.0;
                    expected_torque_max = et + 10.0;
                }
            #elif (NODE_ID == NODE_A3)
                //expected_torque = Dynamics::torque_a3(a1_pos, a2_pos, state.position);
                expected_torque = 0.0;
                expected_torque_min = -10.0;
                expected_torque_max = 10.0;
            #endif
        }

        pidPos.Compute();

        if (!trajectory.complete()) {
            pidVelInput = state.velocity;
            pidVelSetpoint = trajectory.getTargetVelocity();
            step_velocity(false);
        } else {
            pidVelOutput = 0;
            pidVel.clear();
        }

        pidTrqSetpoint = pidPosOutput + (expected_torque * 0.50) + (state.torque * 0.50);
        step_torque(false);
    #endif
}

void Controller::step_velocity (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_GREEN);
    }

    #if (NODE_ID != NODE_G0)
        pidVelInput = state.velocity;
        pidVelSetpoint = constrain(pidVelSetpoint, MIN_VELOCITY, MAX_VELOCITY);
        pidVel.Compute();

        // disable vel controller and wind-down if torque outside bounds
        if (state.torque <= MIN_TORQUE || state.torque >= MAX_TORQUE) {
            pidVelOutput = 0;
            pidVel.clear();
        }

        pidPwrSetpoint = pidVelOutput;
    #endif
}

void Controller::step_torque (bool pulseLED) {
    if (pulseLED == true) {
        led.pulse(LED_YELLOW);
    }

    #if (NODE_ID != NODE_G0)
        pidTrqInput = state.torque;
        pidTrqSetpoint = constrain(pidTrqSetpoint, MIN_TORQUE, MAX_TORQUE);
        pidTrqSetpoint = constrain(pidTrqSetpoint, expected_torque_min, expected_torque_max);
        pidTrq.Compute();

        pidPwrSetpoint = pidVelOutput + pidTrqOutput;
    #endif
}

void Controller::step_motor () {
    pidPwrSetpoint = constrain(pidPwrSetpoint, DEFAULT_MOTOR_MIN, DEFAULT_MOTOR_MAX);
    motor.step(pidPwrSetpoint * 100.0);
}

bool calibrate_forward = true;
int iterations = 0;
int max_iterations = 5;

void Controller::step_calibrate() {
    /*if (iterations >= max_iterations) {
        led.blink(0, 255, 0);
        step_servo(false);
        step_motor();
        return;
    } else {
        led.blink(255, 255, 0);
    }

    if (trajectory.complete()) {
        if (calibrate_forward) {
            calibrate_forward = false;
            trajectory.begin(-3.14, 10000);
            iterations++;
            if (iterations >= max_iterations) {
                trajectory.begin(0, 5000);

                double values[3];
                regression.getValues(values);
                SEA_COEFF_M = values[0];
                SEA_COEFF_B = values[1];

                Serial.print("M: ");
                Serial.print(SEA_COEFF_M);
                Serial.print(", B: ");
                Serial.println(SEA_COEFF_B);
            }
        } else {
            calibrate_forward = true;
            trajectory.begin(3.14, 10000);
        }
    }
    
    step_servo(false);
    step_motor();*/

    /*if (calibrateTimer.ready()) {
         // add accel of weight (with respect to gravity)
         // based on rotational accel
        float grv_acc = 9.8; // m/s/s

        float arm_len = 0.125; // m
        float arm_mss = 0.293; // kg
        float arm_dst = sin(pidPosInput) * arm_len; // m
        float arm_pos_y0 = cos(encoderOutput.getAngleRadians(0)) * arm_len;
        float arm_pos_y1 = cos(encoderOutput.getAngleRadians(1)) * arm_len;
        float arm_pos_y2 = cos(encoderOutput.getAngleRadians(2)) * arm_len;
        float arm_vel_y0 = arm_pos_y0 - arm_pos_y1 / (encoderOutput.getPrevPositionTS(0) / 1000000.0); // m/s
        float arm_vel_y1 = arm_pos_y1 - arm_pos_y2 / (encoderOutput.getPrevPositionTS(1) / 1000000.0); // m/s
        float arm_acc = arm_vel_y0 - arm_vel_y1; // m/s/s
        float arm_trq = arm_dst * (arm_mss * (grv_acc - arm_acc)); // N-m

        float wgt_len = 0.250; // m
        //float wgt_mss = 18.1437; // kg, 40 lbs
        float wgt_mss = 1.300; // kg
        float wgt_dst = sin(pidPosInput) * wgt_len; // m
        float wgt_pos_y0 = cos(encoderOutput.getAngleRadians(0)) * wgt_len;
        float wgt_pos_y1 = cos(encoderOutput.getAngleRadians(1)) * wgt_len;
        float wgt_pos_y2 = cos(encoderOutput.getAngleRadians(2)) * wgt_len;
        float wgt_vel_y0 = wgt_pos_y0 - wgt_pos_y1 / (encoderOutput.getPrevPositionTS(0) / 1000000.0); // m/s
        float wgt_vel_y1 = wgt_pos_y1 - wgt_pos_y2 / (encoderOutput.getPrevPositionTS(1) / 1000000.0); // m/s
        float wgt_acc = wgt_vel_y0 - wgt_vel_y1; // m/s/s
        float wgt_trq = wgt_dst * (wgt_mss * (grv_acc - wgt_acc)); // N-m

        expected_torque = arm_trq + wgt_trq;

        Serial.print(iterations);
        Serial.print(", ");
        Serial.print(encoderTorque.getAngleRadians(), 6);
        Serial.print(", ");
        Serial.print(encoderOutput.getAngleRadians(), 6);
        Serial.print(", ");
        Serial.print(pidPwrSetpoint, 6);
        Serial.print(", ");
        Serial.print(arm_acc, 6);
        Serial.print(", ");
        Serial.print(wgt_acc, 6);
        Serial.print(", ");
        Serial.println(expected_torque, 6);

        //regression.learn(encoderTorque.getAngleRadians(), expected_torque);
    }*/

    if (calibrateTimer.ready()) {
        Serial.print(encoderTorque.getAngleRadians(), 6);
        Serial.print(", ");
        Serial.println(state.torque);
    }
}

void Controller::step_stop () {
    led.pulse(LED_RED);
    pidPos.clear();
    pidVel.clear();
    pidTrq.clear();
    motor.stop();
}