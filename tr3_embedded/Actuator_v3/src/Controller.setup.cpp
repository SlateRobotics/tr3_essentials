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
    
    pidPos.SetMode(AUTOMATIC);
    pidPos.SetOutputLimits(VELOCITY_MIN, VELOCITY_MAX);
    
    pidVel.SetMode(AUTOMATIC);
    pidVel.SetOutputLimits(TORQUE_MIN, TORQUE_MAX);
    pidVel.SetIThresh(TORQUE_MAX);
    pidVel.DisableIClamp();
    
    pidTrq.SetMode(AUTOMATIC);
    pidTrq.SetOutputLimits(-1, 1);
    pidTrq.SetIThresh(1.0);
    pidTrq.DisableIClamp();

    setUpConfig();
    setUpImu();
    step_imu();
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
    if (!storage.begin()) {
        Serial.println("Failed to initialise EEPROM");
        Serial.println("Restarting in 3000 ms...");
        long start = millis();
        while (millis() - start < 3000) { led.blink(255, 0, 0); delay(50); };
        ESP.restart();
    }

    if (storage.isConfigured()) {
        SEA_SPRING_RATE = storage.readUInt16(EEADDR_SEA_SPRING_RATE);

        if (ACTUATOR_ID == "g0") {
            state.position = storage.readFloat(EEADDR_ENC_OUT_POS);
        }

        encoderTorque.configure();
        encoderOutput.configure();
        
        if (storage.readBool(EEADDR_MTR_FLIP)) {
            motor.flipDrivePins();
        }

        double p_pos = storage.readFloat(EEADDR_PID_POS_P);
        double p_vel = storage.readFloat(EEADDR_PID_VEL_P);
        double i_vel = storage.readFloat(EEADDR_PID_VEL_I);
        pidPos.SetTunings(p_pos, 0.0, 0.0);
        pidVel.SetTunings(p_vel, i_vel, 0.0);
    } else {
        storage.configure();
    }
}