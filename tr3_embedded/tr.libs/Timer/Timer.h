#ifndef TIMER_H
#define TIMER_H

class Timer {
  private:
    unsigned long lastEvent = 0;
    double hertz = 20;
    int delayMs = 50;

  public:
    Timer (double h) {
      hertz = h;
      delayMs = int(1000 / hertz);
    }

    bool ready () {
      bool result = (millis() - lastEvent >= delayMs);
      if (result == true) {
        lastEvent = millis();
      }
      return result;
    }

    void wait () {
      while (!ready()) { }
      
    }
    
};

#endif
