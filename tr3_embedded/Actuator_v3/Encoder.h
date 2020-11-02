#ifndef EMS22A_H
#define EMS22A_H

class Encoder {
  private:
    int PIN_CLOCK;
    int PIN_DATA;
    int PIN_CS;

    uint16_t encoderResolution = 4096;
    static const int prevPositionN = 2;
    double prevPosition[prevPositionN];
    long prevPositionTS[prevPositionN];
    double pos = 0;
    double ratio = 1.0;
    uint16_t offset = 0;

    double inl = 0.0139626; // integral non-linearity

    int rotations = 0;

    void formatPosition() {
      double maxPos = ratio * (double)encoderResolution;
      if (pos > maxPos) {
        rotations += 1;
        pos -= maxPos;
      } else if (pos < 0) {
        rotations -= 1;
        pos += maxPos;
      }
    }
  
  public:
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

      if (ACTUATOR_ID == "a0" || ACTUATOR_ID == "a1" || ACTUATOR_ID == "a2" || ACTUATOR_ID == "b0" || ACTUATOR_ID == "b1") {
        ratio = 124.0 / 25.0;
      } else {
        ratio = 104.0 / 25.0;
      }
      
      readPosition();
      readPosition();
      pos = prevPosition[0];
    }
    
    void setOffset(uint16_t o) {
      offset = o;
    }
    
    int readPosition() {
      unsigned int dataOut = 0;
      digitalWrite(PIN_CS, LOW);
      delayMicroseconds(1
      );
    
      for(int x = 0; x < 12; x++){
        digitalWrite(PIN_CLOCK, LOW);
        delayMicroseconds(1);
        digitalWrite(PIN_CLOCK, HIGH);
        delayMicroseconds(1);
        dataOut = (dataOut << 1) | digitalRead(PIN_DATA);
      }
    
      digitalWrite(PIN_CS, HIGH);
      
      prevPosition[1] = prevPosition[0];
      prevPosition[0] = dataOut;

      prevPositionTS[1] = prevPositionTS[0];
      prevPositionTS[0] = millis();
      delayMicroseconds(1);
      
      int16_t dif = prevPosition[0] - prevPosition[1];
      if (dif < -encoderResolution / 2) {
        dif = encoderResolution + dif;
      } else if (dif > encoderResolution / 2) { 
        dif = dif - encoderResolution;
      }

      pos += dif;
      formatPosition();
      
      return dataOut;
    }
    
    void step() {
      readPosition();
    }

    bool isUp() {
      return (prevPosition[0] > encoderResolution / 2.0);
    }

    double getLap () {
      return fmod(ratio + ((pos - (double)prevPosition[0] + (double)offset) / encoderResolution), ratio);
    }

    double getEncoderResolution () {
      return encoderResolution;
    }

    double getRatio () {
      return ratio;
    }
    
    double getOffset() {
      return offset;
    }
    
    double getPosition() {
      return pos;
    }

    void reconstruct(double lap) {
      if (isnan(lap)) {
        return;
      }
      
      pos = lap * encoderResolution + ((double)prevPosition[0] - (double)offset);
      formatPosition();
    }

    void addPosition (double o) {
      pos += o;
      formatPosition();
    }

    double getPrevPosition(int i = 0) {
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
      double err_estimate = getErrorEstimate();
      return pos / (ratio * encoderResolution) * TAU - err_estimate;
    }

    int getRotations () {
      return rotations;
    }
    
};

#endif
