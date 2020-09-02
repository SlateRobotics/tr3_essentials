#ifndef CONTROLLER_H
#define CONTROLLER_H

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

#define CMD_UPDATE_FIRMWARE_BEGIN 0x01
#define CMD_UPDATE_FIRMWARE 0x02
#define CMD_UPDATE_FIRMWARE_END 0x03
#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_STOP_RELEASE 0x15
#define CMD_STOP_EMERGENCY 0x16
#define CMD_FLIP_MOTOR 0x17
#define CMD_CALIBRATE 0x18
#define CMD_SHUTDOWN 0x19
#define CMD_UPDATE_PID 0x20
#define CMD_SET_VELOCITY 0x21

#define MODE_UPDATE_FIRMWARE 0x01
#define MODE_CALIBRATE 0x02
#define MODE_STOP 0x0F
#define MODE_SERVO 0x10
#define MODE_BACKDRIVE 0x11
#define MODE_ROTATE 0x12
#define MODE_VELOCITY 0x13

#define PIN_MTR_PWM 23 // ENABLE / PWM
#define PIN_MTR_IN1 27 // IN1 / DRIVE 1
#define PIN_MTR_IN2 14 // IN2 / DRIVE 2

#define PIN_END_CS  33
#define PIN_END_CLK 25
#define PIN_END_DO  26

#define PIN_ENO_CS  32
#define PIN_ENO_CLK 22
#define PIN_ENO_DO  34

#define PIN_FAN 2

#include <ESPmDNS.h>
#include <Update.h>

#include "ControllerState.h"
#include "Encoder.h"
#include "LED.h"
#include "Motor.h"
#include "MPU9250.h"
#include "NetworkPacket.h"
#include "PID.h"
#include "Timer.h"
#include "Storage.h"
#include "Regression.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68

int status;

class Controller {
  private:
    int mode = MODE_ROTATE;
    int modePrev = MODE_ROTATE;

    double pidPos = 0;
    double pidOut = 0;
    double pidGoal = 0;
    double pidThreshold = 0.001;
    double pidMaxSpeed = 100;

    static const int velocityReadSize = 20;
    float velocityReads[velocityReadSize];

    float SEA_SPRING_RATE = 882.0; // Newton-Meters per Radian

    bool prevUpDrive = false;
    float prevLapDrive = 0.0;
    bool prevUpOutput = false;
    float prevLapOutput = 0.0;

    long calibrateStart = 0;

    ControllerState state;

    Encoder encoderDrive = Encoder(PIN_END_CS, PIN_END_CLK, PIN_END_DO);
    Encoder encoderOutput = Encoder(PIN_ENO_CS, PIN_ENO_CLK, PIN_ENO_DO);
    LED led;
    Motor motor = Motor(PIN_MTR_PWM, PIN_MTR_IN1, PIN_MTR_IN2);
    PID pid = PID(&pidPos, &pidOut, &pidGoal, 9.0, 5.0, 0.2, DIRECT);
    PID pid_vel = PID(&pidPos, &pidOut, &pidGoal, 0.0, 1.0, 0.0, DIRECT);
    Storage storage;

    Regression regression;

    Timer imuTimer = Timer(5); // hz
    MPU9250 imu = MPU9250(Wire,0x68);

    void computeState () {
      double positionDrive = encoderDrive.getAngleRadians();
      double positionOutput = encoderOutput.getAngleRadians();

      float f1 = regression.filter1(positionOutput);
      float f2 = regression.filter2(positionOutput);

      double positionDiff = atan2(sin(positionOutput-positionDrive), cos(positionOutput-positionDrive));
      double positionDiffFiltered = positionDiff;// - f1 - f2;

      bool upDrive = encoderDrive.isUp();
      if (prevUpDrive != upDrive) {
        Serial.print("updating eeprom, encoder drive up: ");
        Serial.println(upDrive);
        storage.writeBool(EEADDR_ENC_D_UP, upDrive);
        storage.commit();
        prevUpDrive = upDrive;
      }

      double lapDrive = encoderDrive.getLap();
      if (abs(prevLapDrive - lapDrive) > 0.01) {
        storage.writeDouble(EEADDR_ENC_D_LAP, lapDrive);
        storage.commit();
        prevLapDrive = lapDrive;
      }

      bool upOutput = encoderOutput.isUp();
      if (prevUpOutput != upOutput) {
        Serial.print("updating eeprom, encoder output up: ");
        Serial.println(upOutput);
        storage.writeBool(EEADDR_ENC_O_UP, upOutput);
        storage.commit();
        prevUpOutput = upOutput;
      }

      double lapOutput = encoderOutput.getLap();
      if (abs(prevLapOutput - lapOutput) > 0.01) {
        storage.writeDouble(EEADDR_ENC_O_LAP, lapOutput);
        storage.commit();
        prevLapOutput = lapOutput;
      }

      state.position = positionOutput;
      state.rotations = encoderOutput.getRotations();
      state.effort = motor.getEffort();
      state.torque = positionDiffFiltered * SEA_SPRING_RATE;

      long timeDiff = encoderOutput.getPrevPositionTS(0) - encoderOutput.getPrevPositionTS(1);
      double dif = encoderOutput.getPrevPosition(0) - encoderOutput.getPrevPosition(1);
      if (dif < -encoderOutput.getEncoderResolution() / 2) {
        dif = encoderOutput.getEncoderResolution() + dif;
      } else if (dif > encoderOutput.getEncoderResolution() / 2) {
        dif = dif - encoderOutput.getEncoderResolution();
      }

      double posDiff = dif / (encoderOutput.getRatio() * encoderOutput.getEncoderResolution()) * TAU;

      float vel = posDiff / (timeDiff / 1000.0);
      float velSum = vel;

      for (int i = velocityReadSize; i > 0; i--) {
        velocityReads[i] = velocityReads[i - 1];
        velSum += velocityReads[i];
      }
      velocityReads[0] = vel;

      state.velocity = velSum / velocityReadSize;

      if (imuTimer.ready()) {
        step_imu();
      }
    }

    double formatAngle (double x) {
      if (x < -TAU) {
        x += TAU;
        return formatAngle(x);
      } else if (x > TAU) {
        x -= TAU;
        return formatAngle(x);
      } else {
        return x;
      }
    }

    void setUpConfig () {
      if (!storage.setUp()) {
        Serial.println("Failed to initialise EEPROM");
        while (1) { led.blink(255, 0, 0); delay(50); };
      }

      if (storage.isConfigured()) {
        SEA_SPRING_RATE = storage.readUInt16(EEADDR_SEA_SPRING_RATE);

        encoderDrive.readPosition();
        int encDriveOffset = storage.readUInt16(EEADDR_ENC_D_OFFSET);
        double encDriveLap = storage.readDouble(EEADDR_ENC_D_LAP);
        bool encDriveUp = storage.readBool(EEADDR_ENC_D_UP);

        if (encDriveUp == true && encoderDrive.isUp() == false) {
          encDriveLap += 1;
        } else if (encDriveUp == false && encoderDrive.isUp() == true) {
          encDriveLap -= 1;
        }

        encoderDrive.setOffset(encDriveOffset);
        encoderDrive.reconstruct(encDriveLap);
        prevLapDrive = encoderDrive.getLap();
        prevUpDrive = encoderDrive.isUp();

        encoderOutput.readPosition();
        int encOutputOffset = storage.readUInt16(EEADDR_ENC_O_OFFSET);
        double encOutputLap = storage.readDouble(EEADDR_ENC_O_LAP);
        bool encOutputUp = storage.readBool(EEADDR_ENC_O_UP);

        if (encOutputUp == true && encoderOutput.isUp() == false) {
          encOutputLap += 1;
        } else if (encOutputUp == false && encoderOutput.isUp() == true) {
          encOutputLap -= 1;
        }

        encoderOutput.setOffset(encOutputOffset);
        encoderOutput.reconstruct(encOutputLap);
        prevLapOutput = encoderOutput.getLap();
        prevUpOutput = encoderOutput.isUp();

        bool mtrFlip = storage.readBool(EEADDR_MTR_FLIP);
        if (mtrFlip == true) {
          motor.flipDrivePins();
        }

        regression.coefficients1[0] = storage.readFloat(EEADDR_REG_C1_1);
        regression.coefficients1[1] = storage.readFloat(EEADDR_REG_C1_2);
        regression.coefficients1[2] = storage.readFloat(EEADDR_REG_C1_3);

        regression.coefficients2[0] = storage.readFloat(EEADDR_REG_C2_1);
        regression.coefficients2[1] = storage.readFloat(EEADDR_REG_C2_2);
        regression.coefficients2[2] = storage.readFloat(EEADDR_REG_C2_3);

        regression.setOffset(storage.readFloat(EEADDR_REG_OFFSET));

        double p = storage.readFloat(EEADDR_PID_P);
        double i = storage.readFloat(EEADDR_PID_I);
        double d = storage.readFloat(EEADDR_PID_D);
        pid.SetTunings(p, i, d);
      } else {
        storage.configure();
      }
    }

    void setUpImu() {
      int imuStatus = imu.begin(19, 18);
      if (imuStatus < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);

        if (requireImu == true) {
          while(1) { led.blink(255, 165, 0); delay(50); }
        }
      }
    }

  public:

    Controller () { }

    bool requireImu = true;

    void setUp () {
      led.setUp();
      motor.setUp();
      encoderDrive.setUp();
      encoderOutput.setUp();
      led.white();
      setUpConfig();
      pinMode(PIN_FAN, OUTPUT);
      setUpImu();
      step_imu();
      fanOff();
      pid.SetMode(AUTOMATIC);
      pid.SetOutputLimits(-1, 1);

      pid_vel.SetMode(AUTOMATIC);
      pid_vel.SetOutputLimits(-1, 1);
      pid_vel.SetIThresh(1.0);
      pid_vel.DisableIClamp();
    }

    ControllerState* getState () {
      return &state;
    }

    void fanOn () {
      digitalWrite(PIN_FAN, LOW);
    }

    void fanOff () {
      digitalWrite(PIN_FAN, HIGH);
    }

    void printData () {
      Serial.print(encoderDrive.readPosition());
      Serial.print(", ");
      Serial.print(encoderDrive.getLap(), 4);
      Serial.print(", ");
      Serial.print(encoderDrive.getOffset());
      Serial.print(", ");
      Serial.print(encoderDrive.getPosition(), 4);
      Serial.print(", ");
      Serial.print(encoderDrive.getAngleRadians(), 4);
      Serial.print("::");
      Serial.print(encoderOutput.readPosition());
      Serial.print(", ");
      Serial.print(encoderOutput.getLap(), 4);
      Serial.print(", ");
      Serial.print(encoderOutput.getOffset());
      Serial.print(", ");
      Serial.print(encoderOutput.getPosition(), 4);
      Serial.print(", ");
      Serial.println(encoderOutput.getAngleRadians(), 4);
    }

    /// ----------------------
    /// --- STEP FUNCTIONS ---
    /// ----------------------

    void step () {
      encoderDrive.step();
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
        step_backdrive();
      } else if (mode == MODE_SERVO) {
        step_servo();
      } else if (mode == MODE_VELOCITY) {
        step_velocity();
      } else if (mode == MODE_CALIBRATE) {
        step_calibrate();
      } else {
        step_stop();
      }
    }

    void step_imu () {
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

    void step_rotate () {
      led.pulse(LED_CYAN);
      motor.executePreparedCommand();
    }

    void step_backdrive () {
      led.pulse(LED_YELLOW);
      if (abs(state.torque) < 4) {
        motor.step(0);
      } else {
        int m = state.torque * 15;
        if (m < -100) {
          m = -100;
        } else if (m > 100) {
          m = 100;
        }
        motor.step(m);
      }
    }

    void step_servo () {
      led.pulse(LED_MAGENTA);
      pidPos = state.position;

      // ignore if within threshold of goal -- good 'nuff
      if (abs(pidPos - pidGoal) >= pidThreshold) {
        double d = formatAngle(pidPos) - formatAngle(pidGoal);
        if (d > PI) {
          pidGoal = formatAngle(pidGoal);
          pidGoal += TAU;
        } else if (d < -PI) {
          pidPos = formatAngle(pidPos);
          pidPos += TAU;
        } else {
          pidGoal = formatAngle(pidGoal);
          pidPos = formatAngle(pidPos);
        }

        pid.Compute();
        int speed = pidOut * 100.0;
        if (speed > pidMaxSpeed) {
          speed = pidMaxSpeed;
        } else if (speed < -pidMaxSpeed) {
          speed = -pidMaxSpeed;
        }
        motor.step(speed);

        Serial.print("GOAL: ");
        Serial.print(pidGoal, 6);
        Serial.print(", POS: ");
        Serial.print(pidPos, 6);
        Serial.print(", EFF: ");
        Serial.println(speed);
      } else {
        motor.stop();
      }
    }

    void step_velocity () {
      led.pulse(LED_MAGENTA);
      pidPos = state.velocity;

      if (abs(pidGoal) > 0.01) {
        pid_vel.Compute();

        Serial.print(pidPos);
        Serial.print(", ");
        Serial.print(pidGoal);
        Serial.print(", ");
        Serial.println(pidOut);

        int speed = pidOut * 100.0;
        if (speed > pidMaxSpeed) {
          speed = pidMaxSpeed;
        } else if (speed < -pidMaxSpeed) {
          speed = -pidMaxSpeed;
        }
        motor.step(speed);
      } else {
        pid_vel.clear();
        motor.stop();
      }
    }

    void step_calibrate() {
      led.pulse(LED_GREEN);

      long duration = 30000;
      if (ACTUATOR_ID == "a0" || ACTUATOR_ID == "a1" || ACTUATOR_ID == "a2" || ACTUATOR_ID == "b0" || ACTUATOR_ID == "b1") {
        duration = 15000;
      }

      if (millis() - calibrateStart < duration) {
        encoderDrive.step();
        encoderOutput.step();
        motor.executePreparedCommand();

        float positionDrive = encoderDrive.getAngleRadians();
        float positionOutput = encoderOutput.getAngleRadians();
        float positionDiff = atan2(sin(positionOutput-positionDrive), cos(positionOutput-positionDrive));
        regression.add(positionOutput, positionDiff);
      } else {
        regression.train();

        storage.writeFloat(EEADDR_REG_C1_1, regression.coefficients1[0]);
        storage.writeFloat(EEADDR_REG_C1_2, regression.coefficients1[1]);
        storage.writeFloat(EEADDR_REG_C1_3, regression.coefficients1[2]);

        storage.writeFloat(EEADDR_REG_C2_1, regression.coefficients2[0]);
        storage.writeFloat(EEADDR_REG_C2_2, regression.coefficients2[1]);
        storage.writeFloat(EEADDR_REG_C2_3, regression.coefficients2[2]);

        storage.writeFloat(EEADDR_REG_OFFSET, regression.getOffset());
        storage.commit();

        mode = MODE_ROTATE;
      }
    }

    void step_stop () {
      led.pulse(LED_RED);
      motor.stop();
    }

    /// ---------------------
    /// --- CMD FUNCTIONS ---
    /// ---------------------

    void parseCmd (NetworkPacket packet) {
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
      } else if (packet.command == CMD_UPDATE_PID) {
        cmd_updatePid(packet);
      } else if (packet.command == CMD_SET_VELOCITY) {
        cmd_setVelocity(packet);
      }
    }

    void cmd_setMode (NetworkPacket packet) {
      mode = packet.parameters[0];
      if (mode == MODE_SERVO) {
        pidMaxSpeed = 100;
        pidGoal = (double)state.position;
      } else if (mode == MODE_VELOCITY) {
        pidGoal = 0;
      }
    }

    void cmd_setPosition (NetworkPacket packet) {
      int param = packet.parameters[0] + packet.parameters[1] * 256;
      pidMaxSpeed = floor(100.0 * packet.parameters[2] / 255.0);
      double pos = param / 65535.0 * TAU;
      Serial.println(pos);
      pidGoal = formatAngle(pos);
    }

    void cmd_setVelocity (NetworkPacket packet) {
      int param = packet.parameters[0] + packet.parameters[1] * 256;
      pidGoal = (param / 100.0) - 10.0;
      Serial.print(packet.parameters[0]);
      Serial.print(",");
      Serial.print(packet.parameters[1]);
      Serial.print(":");
      Serial.println(pidGoal);
    }

    void cmd_resetPosition () {
      float positionOutput = encoderOutput.getAngleRadians();
      regression.addOffset(positionOutput);

      float pos = encoderOutput.getPosition();
      encoderOutput.addPosition(-pos);
      encoderDrive.addPosition(-pos);

      pidGoal = 0;
      pidOut = 0;
      pidPos = 0;
      pid.clear();

      uint16_t encO_offset = encoderOutput.getOffset();
      uint16_t encD_offset = encoderDrive.getOffset();
      storage.writeUInt16(EEADDR_ENC_O_OFFSET, encO_offset);
      storage.writeUInt16(EEADDR_ENC_D_OFFSET, encD_offset);

      storage.writeFloat(EEADDR_REG_C1_1, regression.coefficients1[0]);
      storage.writeFloat(EEADDR_REG_C1_2, regression.coefficients1[1]);
      storage.writeFloat(EEADDR_REG_C1_3, regression.coefficients1[2]);

      storage.writeFloat(EEADDR_REG_C2_1, regression.coefficients2[0]);
      storage.writeFloat(EEADDR_REG_C2_2, regression.coefficients2[1]);
      storage.writeFloat(EEADDR_REG_C2_3, regression.coefficients2[2]);

      storage.writeFloat(EEADDR_REG_OFFSET, regression.getOffset());
      storage.commit();
    }

    void cmd_flipMotorPins () {
      motor.flipDrivePins();

      storage.writeBool(EEADDR_MTR_FLIP, motor.flipDrivePinsStatus());
      storage.commit();
    }

    void cmd_rotate (NetworkPacket packet) {
      int offsetBinary = 128;
      int motorStep = packet.parameters[0] - offsetBinary;
      int motorDuration = packet.parameters[1] + packet.parameters[2] * 256;

      if (motorDuration > 1000) {
        motorDuration = 1000;
      }

      motor.prepareCommand(motorStep, motorDuration);
    }

    void cmd_release () {
      mode = modePrev;
    }

    void cmd_stop () {
      if (mode != MODE_STOP) {
        modePrev = mode;
      } else {
        modePrev = MODE_ROTATE;
      }
      mode = MODE_STOP;
    }

    void cmd_calibrate() {
      calibrateStart = millis();
      mode = MODE_CALIBRATE;
      encoderOutput.resetPos();
      encoderDrive.resetPos();

      if (ACTUATOR_ID == "a0" || ACTUATOR_ID == "a1" || ACTUATOR_ID == "a2" || ACTUATOR_ID == "b0" || ACTUATOR_ID == "b1") {
      SEA_SPRING_RATE = 882;
      } else {
      SEA_SPRING_RATE = 300;
      }
      storage.writeUInt16(EEADDR_SEA_SPRING_RATE, SEA_SPRING_RATE);

      uint16_t encO_offset = encoderOutput.getOffset();
      uint16_t encD_offset = encoderDrive.getOffset();
      storage.writeUInt16(EEADDR_ENC_O_OFFSET, encO_offset);
      storage.writeUInt16(EEADDR_ENC_D_OFFSET, encD_offset);
      storage.commit();
    }

    void cmd_shutdown() {
      led.off();
      motor.stop();
      computeState();

      storage.writeBool(EEADDR_ENC_O_UP, encoderOutput.isUp());
      storage.writeDouble(EEADDR_ENC_O_LAP, encoderOutput.getLap());
      storage.writeBool(EEADDR_ENC_D_UP, encoderDrive.isUp());
      storage.writeDouble(EEADDR_ENC_D_LAP, encoderDrive.getLap());
      storage.commit();

      while (1) {
        motor.stop();
      }
    }

    void cmd_updatePid(NetworkPacket packet) {
      double p = packet.parameters[0] / 10.0;
      double i = packet.parameters[1] / 10.0;
      double d = packet.parameters[2] / 10.0;
      pid.SetTunings(p, i, d);
      storage.writeFloat(EEADDR_PID_P, p);
      storage.writeFloat(EEADDR_PID_I, i);
      storage.writeFloat(EEADDR_PID_D, d);
      storage.commit();
    }
};

#endif
