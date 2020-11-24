#ifndef PID_v1_h
#define PID_v1_h

class PID {
  private:
    double kp;
    double ki;
    double kd;

    double *input;
    double *output;
    double *setpoint;

    double feedforward;
    double outputSum;

    double outMin;
    double outMax;
    double iClamp = 0.25;

    unsigned long lastTime;
    double lastInput;
    double lastSetpoint;

  public:
    PID () { }
    
    PID(double* _input, double* _output, double* _setpoint, double _kp, double _ki, double _kd) {
      output = _output;
      input = _input;
      setpoint = _setpoint;
    
      PID::SetOutputLimits(-1.0, 1.0);
      PID::SetTunings(_kp, _ki, _kd);
    
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
      outputSum = constrain(outputSum, -iClamp, iClamp);
  
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

    double GetTunings(int i) {
      double gains[3];
      gains[0] = kp;
      gains[1] = ki;
      gains[2] = kd;
      return gains[i];
    }
    
    void SetTunings(double Kp, double Ki, double Kd) {
      kp = Kp;
      ki = Ki;
      kd = Kd;
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
