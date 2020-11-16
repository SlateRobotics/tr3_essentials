#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESPmDNS.h>
#include <Update.h>

#include "Config.h"
#include "LED.h"

int status;

class Controller {
  private:
    bool powerOn = false;
    LED led;
  
  public:
    Controller () { }

    void setUp () {
      led.setUp();
      pinMode(PIN_RELAY, OUTPUT);
      led.white();
    }

    bool poweredOn () {
      return powerOn;
    }

    /// ----------------------
    /// --- STEP FUNCTIONS ---
    /// ----------------------
    void step () {
      if (powerOn == true) {
        led.pulse(LED_GREEN);
        digitalWrite(PIN_RELAY, HIGH);
      } else {
        led.pulse(LED_RED);
        digitalWrite(PIN_RELAY, LOW);
      }
    }

    /// ---------------------
    /// --- CMD FUNCTIONS ---
    /// ---------------------
    void cmd_poweron () {
      powerOn = true;
    }

    void cmd_poweroff () {
      powerOn = false;
    }
};

#endif
