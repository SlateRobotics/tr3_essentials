#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION  1.2.1

#define AUTOMATIC 1
#define MANUAL  0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1

class PID {
  private:
    double dispKp;
    double dispKi;
    double dispKd;
    double kp;
    double ki;
    double kd;
    int controllerDirection;
    int pOn;
    double *myInput;
    double *myOutput;
    double *mySetpoint;
    double feedforward;
    unsigned long lastTime;
    double outputSum, lastInput, lastSetpoint;
    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto, pOnE;
    double iClamp = 0.25;
    
    void Initialize () {
      outputSum = *myOutput;
      lastInput = *myInput;
      if (outputSum > outMax) outputSum = outMax;
      else if (outputSum < outMin) outputSum = outMin;
    }

  public:
    PID () { }
    
    PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection) {
      myOutput = Output;
      myInput = Input;
      mySetpoint = Setpoint;
      inAuto = false;
    
      PID::SetOutputLimits(0, 255);
      SampleTime = 100;
      
      PID::SetControllerDirection(ControllerDirection);
      PID::SetTunings(Kp, Ki, Kd);
    
      lastTime = millis() - SampleTime;
    }

    
    bool Compute () {
      if (!inAuto) return false;
      double input = *myInput;
      double setpoint = *mySetpoint;
      double error = *mySetpoint - input;
      double dInput = input - lastInput;
      double output = 0;
  
      outputSum += (ki * error);

      outputSum = constrain(outputSum, -iClamp, iClamp);
  
      output += setpoint * feedforward;
      output += kp * error;
      output += outputSum;
      output -= kd * dInput;
      
      if (output > outMax) output = outMax;
      else if (output < outMin) output = outMin;
      *myOutput = output;
  
      lastSetpoint = setpoint;
      lastInput = input;
      lastTime = millis();
      return true;
    }

    void SetMode(int Mode) {
      bool newAuto = (Mode == AUTOMATIC);
      if (newAuto && !inAuto) {
        PID::Initialize();
      }
      inAuto = newAuto;
    }

    void SetOutputLimits(double Min, double Max) {
      if (Min >= Max) return;
      outMin = Min;
      outMax = Max;
    
      if (inAuto) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;
    
        if (outputSum > outMax) outputSum = outMax;
        else if (outputSum < outMin) outputSum = outMin;
      }
    }

    double GetTunings(int i) {
      double gains[3];
      gains[0] = kp;
      gains[1] = ki;
      gains[2] = kd;
      return gains[i];
    }
    
    void SetTunings(double Kp, double Ki, double Kd) {
      SetTunings(Kp, Ki, Kd, pOn);
    }
    
    void SetTunings(double Kp, double Ki, double Kd, int POn) {
      if (Kp < 0 || Ki < 0 || Kd < 0) return;
    
      pOn = POn;
      pOnE = POn == P_ON_E;
    
      dispKp = Kp; dispKi = Ki; dispKd = Kd;
    
      double SampleTimeInSec = ((double)SampleTime) / 1000;
      kp = Kp;
      ki = Ki * SampleTimeInSec;
      kd = Kd / SampleTimeInSec;
    
      if (controllerDirection == REVERSE) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
      }
    }

    void SetFeedforward (double ff) {
      feedforward = ff;
    }
    
    void SetControllerDirection(int Direction) {
      if (inAuto && Direction != controllerDirection) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
      }
      controllerDirection = Direction;
    }

    void SetIClamp(float c) {
      iClamp = c;
    }
    
    void clear() {
      outputSum = 0;
    }
};

#endif
