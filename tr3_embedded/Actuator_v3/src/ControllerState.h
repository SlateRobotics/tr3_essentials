#ifndef CONTROLLERSTATE_H
#define CONTROLLERSTATE_H

class ControllerState {
  private:

  public:
    int mode = 0; // effort, servo, backdrive, velocity
    int stop = 0;
    double position = 0.0; // Radians
    int rotations = 0;
    double effort = 0.0; // -100 -> 100
    double velocity = 0.0; // Radians per second
    double acceleration = 0.0;
    double torque = 0.0; // Newton-Meters
    double accel[3] = {0, 0, 0}; // x, y, z (meter/sec^2)
    double gyro[3] = {0, 0, 0}; // x, y, z (rad/sec)
    double mag[3] = {0, 0, 0}; // x, y, z (uT, microtesla)
    double temp = 0.0; // celsius

    void compute () {
      
    }
  
    String toString() {
      String result = "";
      result += String(position, 6);
      result += ",";
      result += rotations;
      result += ",";
      result += effort;
      result += ",";
      result += velocity;
      result += ",";
      result += torque;
      result += ",";
      result += mode;
      result += ",";
      result += stop;
      result += ",";
      result += temp;
      /*result += ",accel:{";
      result += accel[0];
      result += ",";
      result += accel[1];
      result += ",";
      result += accel[2];
      result += "},gyro:{";
      result += gyro[0];
      result += ",";
      result += gyro[1];
      result += ",";
      result += gyro[2];
      result += "},mag:{";
      result += mag[0];
      result += ",";
      result += mag[1];
      result += ",";
      result += mag[2];
      result += "},temp:";*/
      result += ";";
      return result;
    }
};

#endif
