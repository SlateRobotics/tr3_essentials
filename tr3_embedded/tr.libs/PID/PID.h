#ifndef PID_v1_h
#define PID_v1_h

class PID {
  private:
    // gains
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
    double iClamp = 0.25;
    double feedforward = 0.0;

    double *input;
    double *output;
    double *setpoint;

    double outputSum;

    double outMin;
    double outMax;

    unsigned long lastTime;
    double lastInput;
    double lastSetpoint;

  public:
    PID () { }
    
    PID(double* _input, double* _output, double* _setpoint) {
      output = _output;
      input = _input;
      setpoint = _setpoint;
    
      PID::SetOutputLimits(-1.0, 1.0);
    
      lastTime = micros();
    }

    
    bool Compute () {
      double _input = *input;
      double _setpoint = *setpoint;

      double error = *setpoint - _input;
      double dInput = _input - lastInput;
      double dTimeSec = (micros() - lastTime) / 1000000.0;
      dTimeSec = constrain(dTimeSec, 0.0, 1.0);
  
      outputSum += (ki * dTimeSec) * error;

      // iClamp of 0.0 assumes no iClamp
      if (iClamp > 0.01) {
        outputSum = constrain(outputSum, -iClamp, iClamp);
      }
  
      double _output = 0;
      _output += _setpoint * feedforward;
      _output += kp * error;
      _output += outputSum;
      _output -= (kd / dTimeSec) * dInput;
      
      _output = constrain(_output, outMin, outMax);
      *output = _output;
  
      lastSetpoint = _setpoint;
      lastInput = _input;
      lastTime = micros();
      return true;
    }

    void SetOutputLimits(double Min, double Max) {
      outMin = Min;
      outMax = Max;
    }

    double getIntegralSum () {
      return outputSum;
    }

    double GetTunings(int i) {
      double gains[5];
      gains[0] = kp;
      gains[1] = ki;
      gains[2] = kd;
      gains[3] = iClamp;
      gains[4] = feedforward;
      return gains[i];
    }
    
    void SetTunings(int i, double val) {
      switch (i) {
        case 0:
          kp = val;
          break;
        case 1:
          ki = val;
          break;
        case 2:
          kd = val;
          break;
        case 3:
          iClamp = val;
          break;
        case 4:
          feedforward = val;
          break;
      }
    }

    void SetFeedforward (double ff) {
      feedforward = ff;
    }

    void SetIClamp(float c) {
      iClamp = c;
    }
    
    void clear() {
      outputSum = 0;
    }
};

#endif
