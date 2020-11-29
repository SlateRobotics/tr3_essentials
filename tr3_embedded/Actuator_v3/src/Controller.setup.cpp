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
    pidPos.SetIClamp(MAX_TORQUE * 0.25);
    pidPos.SetFeedforward(0.0);
    
    pidVel.SetOutputLimits(DEFAULT_MOTOR_MIN, DEFAULT_MOTOR_MAX);
    pidVel.SetIClamp(0.50);
    pidVel.SetFeedforward(0.2);
    
    pidTrq.SetOutputLimits(DEFAULT_MOTOR_MIN, DEFAULT_MOTOR_MAX);
    pidTrq.SetIClamp(0.50);
    pidTrq.SetFeedforward(0.0);
    
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
    storage.begin();
    if (storage.isConfigured()) {
        SEA_SPRING_RATE = storage.readFloat(EEADDR_SEA_SPRING_RATE);

        if (NODE_ID == "g0") {
            state.position = storage.readFloat(EEADDR_ENC_OUT_POS);
        }

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

        double p_pos = storage.readFloat(EEADDR_PID_POS_P);
        double i_pos = storage.readFloat(EEADDR_PID_POS_I);
        double d_pos = storage.readFloat(EEADDR_PID_POS_D);
        double p_vel = storage.readFloat(EEADDR_PID_VEL_P);
        double i_vel = storage.readFloat(EEADDR_PID_VEL_I);
        double d_vel = storage.readFloat(EEADDR_PID_VEL_D);
        double p_trq = storage.readFloat(EEADDR_PID_TRQ_P);
        double i_trq = storage.readFloat(EEADDR_PID_TRQ_I);
        double d_trq = storage.readFloat(EEADDR_PID_TRQ_D);
        pidPos.SetTunings(p_pos, i_pos, d_pos);
        pidVel.SetTunings(p_vel, i_vel, d_vel);
        pidTrq.SetTunings(p_trq, i_trq, d_trq);
    } else {
        storage.reset();
    }
}