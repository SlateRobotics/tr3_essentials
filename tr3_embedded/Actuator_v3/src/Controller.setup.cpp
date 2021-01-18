#include "Controller.h"

void Controller::setUp () {
    led.setUp();
    motor.setUp();
    led.white();

    pinMode(PIN_FAN, OUTPUT);
    fanOff();

    encoderTorque.setUp();
    encoderTorque.storage = &storage;
    encoderTorque.EEADDR_ENC_OFFSET = EEADDR_ENC_TRQ_OFFSET;
    encoderTorque.EEADDR_ENC_UP = EEADDR_ENC_TRQ_UP;
    encoderTorque.EEADDR_ENC_LAP = EEADDR_ENC_TRQ_LAP;

    encoderOutput.setUp();
    encoderOutput.storage = &storage;
    encoderOutput.EEADDR_ENC_OFFSET = EEADDR_ENC_OUT_OFFSET;
    encoderOutput.EEADDR_ENC_UP = EEADDR_ENC_OUT_UP;
    encoderOutput.EEADDR_ENC_LAP = EEADDR_ENC_OUT_LAP;

    setUpConfig();
    
    pidPos.SetOutputLimits(MIN_TORQUE, MAX_TORQUE);
    pidVel.SetOutputLimits(DEFAULT_MOTOR_MIN, DEFAULT_MOTOR_MAX); 
    pidTrq.SetOutputLimits(DEFAULT_MOTOR_MIN, DEFAULT_MOTOR_MAX);
    
    //setUpImu();
    //step_imu();
}

void Controller::setUpImu() {
    int imuStatus = imu.begin(19, 18);
    if (imuStatus < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.println("Restarting in 3000 ms... ");

        if (requireImu == true) {
            long start = millis();
            while(millis() - start < 3000) { led.blink(255, 165, 0); delay(50); }
            ESP.restart();
        }
    }
}

void Controller::setUpConfig () {
    storage.begin(&led);
    if (storage.isConfigured()) {
        SEA_SPRING_RATE = storage.readFloat(EEADDR_SEA_SPRING_RATE);
        SEA_COEFF_M = storage.readFloat(EEADDR_SEA_M);
        SEA_COEFF_B = storage.readFloat(EEADDR_SEA_B);

        #if (NODE_ID == NODE_G0)
            state.position = storage.readFloat(EEADDR_ENC_OUT_POS);
        #endif

        encoderTorque.configure();
        encoderOutput.configure();
        
        if (storage.readBool(EEADDR_MTR_FLIP)) {
            motor.flipDrivePins();
        }

        MIN_POSITION = storage.readFloat(EEADDR_POSITION_MIN);
        MAX_POSITION = storage.readFloat(EEADDR_POSITION_MAX);
        MIN_VELOCITY = storage.readFloat(EEADDR_VELOCITY_MIN);
        MAX_VELOCITY = storage.readFloat(EEADDR_VELOCITY_MAX);
        MIN_TORQUE = storage.readFloat(EEADDR_TORQUE_MIN);
        MAX_TORQUE = storage.readFloat(EEADDR_TORQUE_MAX);
        expected_torque_min = MIN_TORQUE;
        expected_torque_max = MAX_TORQUE;

        pidPos.SetTunings(0, storage.readFloat(EEADDR_PID_POS_P));
        pidPos.SetTunings(1, storage.readFloat(EEADDR_PID_POS_I));
        pidPos.SetTunings(2, storage.readFloat(EEADDR_PID_POS_D));
        pidPos.SetTunings(3, storage.readFloat(EEADDR_PID_POS_IC));
        pidPos.SetTunings(4, storage.readFloat(EEADDR_PID_POS_FF));

        pidVel.SetTunings(0, storage.readFloat(EEADDR_PID_VEL_P));
        pidVel.SetTunings(1, storage.readFloat(EEADDR_PID_VEL_I));
        pidVel.SetTunings(2, storage.readFloat(EEADDR_PID_VEL_D));
        pidVel.SetTunings(3, storage.readFloat(EEADDR_PID_VEL_IC));
        pidVel.SetTunings(4, storage.readFloat(EEADDR_PID_VEL_FF));

        pidTrq.SetTunings(0, storage.readFloat(EEADDR_PID_TRQ_P));
        pidTrq.SetTunings(1, storage.readFloat(EEADDR_PID_TRQ_I));
        pidTrq.SetTunings(2, storage.readFloat(EEADDR_PID_TRQ_D));
        pidTrq.SetTunings(3, storage.readFloat(EEADDR_PID_TRQ_IC));
        pidTrq.SetTunings(4, storage.readFloat(EEADDR_PID_TRQ_FF));
    } else {
        storage.reset();
    }
}