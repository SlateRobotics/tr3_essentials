#ifndef CONTROLLERSTATE_H
#define CONTROLLERSTATE_H

class ControllerState {
  private:

  public:
    float vCell1 = 0.0; // cell 1 voltage
    float vCell2 = 0.0; // cell 2 voltage
    float vCell3 = 0.0; // cell 3 voltage
    float vBattery = 0.0;
  
    String toString() {
      String result = "bat:";
      result += vBattery;
      result += ",cv1:";
      result += vCell1;
      result += ",cv2:";
      result += vCell2;
      result += ",cv3:";
      result += vCell3;
      result += ";";
      return result;
    }
};

#endif
