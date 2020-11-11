#ifndef CONTROLLERSTATE_H
#define CONTROLLERSTATE_H

class ControllerState {
  private:

  public:
    int position = 0;
  
    String toString() {
      String result = "pos:";
      result += position;
      result += ";";
      return result;
    }
};

#endif
