#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    int pinEnable;
    int pinDrive1;
    int pinDrive2;
    bool flagExecute = false;
    int flagExecuteSpeed = 0;
    int flagExecuteDuration = 0;
    int lastPreparedCommand[2] = {0, 0};
    int motorDirection = 0;
    unsigned long flagExecuteExpiration = millis();
    int minSpeed = 10;
    int maxSpeed = 100;
    int effort = 0;
    void setPinSpeed() {
      if (motorSpeed > maxSpeed) {
        motorSpeed = maxSpeed;
      }

      int pinStrength = int(255.0 * (motorSpeed / 100.0));
      ledcWrite(0, pinStrength);
    }
  
  public:
    int id;
    int motorSpeed;
    
    Motor(int pEnable, int pIn1, int pIn2) {
      // motorSpeed set to a percentage of max voltage to pins
      motorSpeed = 0;
      pinEnable = pEnable;
      pinDrive1 = pIn1;
      pinDrive2 = pIn2;
    }
    
    void setUp() {
      ledcSetup(0, 20000, 8);
      ledcAttachPin(pinEnable, 0);
      pinMode(pinDrive1, OUTPUT);
      pinMode(pinDrive2, OUTPUT);
    
      motorSpeed = 0;
      setPinSpeed();
      delay(50);
      stop();
    }

    int getEffort() {
      return effort;
    }

    int flipDrivePinsStatus () {
      if (pinDrive1 == PIN_MTR_IN1) {
        return 0;
      } else {
        return 1;
      }
    }

    void flipDrivePins () {
      int pd1 = pinDrive1;
      pinDrive1 = pinDrive2;
      pinDrive2 = pd1;
    }
    
    void forward(int speed = 100) {
      // given speed should be a percentage of total speed so that we can tell it to start slowly
      motorSpeed = speed;
      effort = speed;
      if (speed < minSpeed) {
        ledcWrite(0, 0);
        digitalWrite(pinDrive1, LOW);
        digitalWrite(pinDrive2, LOW);
      } else {
        setPinSpeed();
        digitalWrite(pinDrive1, HIGH);
        digitalWrite(pinDrive2, LOW);
      }
    }
    
    void backward(int speed = 100) {
      motorSpeed = speed;
      effort = -speed;
      if (speed < minSpeed) {
        ledcWrite(0, 0);
        digitalWrite(pinDrive1, LOW);
        digitalWrite(pinDrive2, LOW);
      } else {
        setPinSpeed();
        digitalWrite(pinDrive1, LOW);
        digitalWrite(pinDrive2, HIGH);
      }
    }
    
    void step(int s = 100) {
      effort = 100;
      if (s > 0) {
        forward(abs(s));
      } else if (s < 0) {
        backward(abs(s));
      } else {
        stop();
      }
    }
    
    void stop() {
      motorSpeed = 0;
      effort = 0;
      ledcWrite(0, 0);
      digitalWrite(pinDrive1, LOW);
      digitalWrite(pinDrive2, LOW);
    }
    
    void getLastMotorCommand(int *cmd) {
      cmd[0] = lastPreparedCommand[0];
      cmd[1] = lastPreparedCommand[1];
    }
    
    bool isFlagged() {
      return flagExecute;
    }
    
    void prepareCommand(int motorSpeed, int duration) {
      flagExecute = true;
      flagExecuteSpeed = motorSpeed;
      flagExecuteDuration = duration;
      flagExecuteExpiration = millis() + duration;
    
      if (flagExecuteExpiration > millis() + 1000) {
        flagExecuteExpiration = millis() + 1000;
      }
    
      lastPreparedCommand[0] = motorSpeed;
      lastPreparedCommand[1] = duration;
    }
    
    void executePreparedCommand() {
      if (flagExecute == true && millis() < flagExecuteExpiration) {
        step(flagExecuteSpeed);
      } else if (millis() > flagExecuteExpiration) {
        step(0);
        flagExecute = false;
      } else {
        step(0);
      }
    }
    
    void clearPreparedCommand() {
      flagExecute = false;
      flagExecuteSpeed = 0;
      flagExecuteDuration = 0;
      flagExecuteExpiration = millis() - 2000;
    }
};

#endif
