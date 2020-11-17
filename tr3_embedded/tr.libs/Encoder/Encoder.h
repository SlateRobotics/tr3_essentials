#ifndef EMS22A_H
#define EMS22A_H

#include "Storage.h"

class Encoder {
  private:
    int PIN_CLOCK;
    int PIN_DATA;
    int PIN_CS;

    uint16_t encoderResolution = 4096;

    static const int prevAngleN = 4;
    double prevAngle[prevAngleN];
    long prevAngleTS[prevAngleN];

    static const int prevPositionN = 16;
    uint16_t prevPosition[prevPositionN];
    long prevPositionTS[prevPositionN];

    static const int prevVelocityN = 4;
    double prevVelocity[prevVelocityN];
    long prevVelocityTS[prevVelocityN];

    double velocity = 0;
    double acceleration = 0;

    double ratio = 1.0;
    uint16_t offset = 0;

    double inl = 0.0139626; // integral non-linearity

    int rotations = 0;

    bool prevUp = false;
    int prevLap = 0;
  
  public:
    int EEADDR_ENC_OFFSET = -1;
    int EEADDR_ENC_UP = -1;
    int EEADDR_ENC_LAP = -1;
    Storage* storage;
    long pos = 0;

    Encoder(int pCs, int pClock, int pData) {
      PIN_CS = pCs;
      PIN_CLOCK = pClock;
      PIN_DATA = pData;
    }
    
    void setUp () {
      pinMode(PIN_CS, OUTPUT);
      pinMode(PIN_CLOCK, OUTPUT);
      pinMode(PIN_DATA, INPUT);
    
      digitalWrite(PIN_CLOCK, HIGH);
      digitalWrite(PIN_CS, HIGH);

      if (NODE_ID == "a0" || NODE_ID == "a1" || NODE_ID == "a2" || NODE_ID == "b0" || NODE_ID == "b1") {
        ratio = 124.0 / 25.0;
      } else {
        ratio = 104.0 / 25.0;
      }
      
      readPosition();
      readPosition();
      pos = prevPosition[0];
    }

    void configure () {
      readPosition();
      int encOffset = storage->readUInt16(EEADDR_ENC_OFFSET);
      double encLap = storage->readDouble(EEADDR_ENC_LAP);
      bool encUp = storage->readBool(EEADDR_ENC_UP);

      if (encUp == true && isUp() == false) {
        encLap += 1;
      } else if (encUp == false && isUp() == true) {
        encLap -= 1;
      }

      setOffset(encOffset);
      reconstruct(encLap);

      prevUp = isUp();
      prevLap = getLap();

      storage->writeBool(EEADDR_ENC_UP, prevUp);
      storage->writeDouble(EEADDR_ENC_LAP, prevLap);
      storage->commit();
    }
    
    void step() {
      readPosition();
      writeChanges();
    }
    
    void setOffset(uint16_t o) {
      offset = o;
    }
    
    int readPosition() {
      unsigned int dataOut = 0;
      digitalWrite(PIN_CS, LOW);
      delayMicroseconds(1);
    
      for(int x = 0; x < 12; x++){
        digitalWrite(PIN_CLOCK, LOW);
        delayMicroseconds(1);
        digitalWrite(PIN_CLOCK, HIGH);
        delayMicroseconds(1);
        dataOut = (dataOut << 1) | digitalRead(PIN_DATA);
      }
    
      digitalWrite(PIN_CS, HIGH);
      delayMicroseconds(1);

      recordPosition(dataOut);
      
      int16_t dif = prevPosition[0] - prevPosition[1];
      if (dif < -encoderResolution / 2) {
        dif = encoderResolution + dif;
      } else if (dif > encoderResolution / 2) { 
        dif = dif - encoderResolution;
      }

      long timeDiff = getPrevPositionTS(0) - getPrevPositionTS(1);
      double posDiff = dif / (getRatio() * getEncoderResolution()) * TAU;
      float vel = posDiff / (timeDiff / 1000.0);
      recordVelocity(vel);

      pos += dif;
      
      double a = getAngleRadians();
      recordAngle();
      
      return dataOut;
    }

    void writeChanges() {
      if (upChanged()) {
        storage->writeBool(EEADDR_ENC_UP, isUp());
        storage->commit();
      }

      if (lapChanged()) {
        storage->writeDouble(EEADDR_ENC_LAP, getLap());
        storage->commit();
      }
    }

    bool lapChanged () {
      int lap = getLap();
      if (abs(prevLap - lap) > 0) {
        prevLap = lap;
        return true;
      } else {
        return false;
      }
    }

    bool upChanged () {
      bool up = isUp();
      if (prevUp != up) {
        prevUp = up;
        return true;
      } else {
        return false;
      }
    }
 
    void recordPosition (unsigned int data) {
      for (int i = prevPositionN - 1; i > 0; i--) {
        prevPosition[i] = prevPosition[i - 1];
        prevPositionTS[i] = prevPositionTS[i - 1];
      }

      prevPosition[0] = data;
      prevPositionTS[0] = millis();
    }

    void recordVelocity (double vel) {
      double velSum = vel;

      for (int i = prevVelocityN - 1; i > 0; i--) {
        prevVelocity[i] = prevVelocity[i - 1];
        prevVelocityTS[i] = prevVelocityTS[i - 1];
        velSum += prevVelocity[i];
      }

      double newVel = velSum / (double)prevVelocityN;
      acceleration = newVel - velocity;
      velocity = newVel;

      prevVelocity[0] = vel;
      prevVelocityTS[0] = millis();
    }

    double getAcceleration () {
      return acceleration;
    }

    double getVelocity () {
      return velocity;
    }

    bool isUp() {
      return (prevPosition[0] > encoderResolution / 2.0);
    }

    int getLap () {
      return (int)floor((double)(pos + offset) / (double)encoderResolution);
    }

    double getEncoderResolution () {
      return encoderResolution;
    }

    double getRatio () {
      return ratio;
    }
    
    uint16_t getOffset() {
      return offset;
    }
    
    double getPosition() {
      return pos;
    }

    void reconstruct(double lap) {
      if (isnan(lap)) {
        return;
      }
      
      pos = (int)(lap * encoderResolution) + prevPosition[0] - offset;
    }

    uint16_t getPrevPosition(int i = 0) {
      return prevPosition[i];
    }

    long getPrevPositionTS(int i) {
      return prevPositionTS[i];
    }

    void resetPos () {
      offset = readPosition();
      pos = 0;
      prevPosition[1] = prevPosition[0];
    }

    double getErrorEstimate() {
      double angle = (double)prevPosition[0] / (double)encoderResolution * PI * 2.0;
      return inl * sin(-angle + PI);
    }
    
    double getAngleRadians() {
      return prevAngle[0];
    }

    void recordAngle () {
      double err_estimate = getErrorEstimate();
      double a = pos / (ratio * encoderResolution) * TAU - err_estimate;
      
      for (int i = prevAngleN - 1; i > 0; i--) {
        prevAngle[i] = prevAngle[i - 1];
        prevAngleTS[i] = prevAngleTS[i - 1];
      }

      prevAngle[0] = a;
      prevAngleTS[0] = millis();
    }

    double getAverageAngle() {
      double sum = 0;
      for (int i = 0; i < prevAngleN; i++) {
        sum += prevAngle[i];
      }
      return sum / (double)prevAngleN;
    }

    int getRotations () {
      return rotations;
    }
    
};

#endif
