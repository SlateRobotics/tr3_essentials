#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESPmDNS.h>
#include <Update.h>

#include "Config.h"
#include "Storage.h"
#include "LED.h"

int status;

class Controller {
  private:
    bool powerOn = false;
    Storage storage;
    LED led;
  
  public:
    Controller () { }

    void setUp () {
      led.setUp();
      pinMode(PIN_RELAY, OUTPUT);

      setUpConfig();

      led.white();
    }

    void setUpConfig () {
        if (!storage.begin()) {
            Serial.println("Failed to initialise EEPROM");
            Serial.println("Restarting in 3000 ms...");
            long start = millis();

            while (millis() - start < 3000) {
              led.blink(255, 0, 0);
              led.step();
              delay(50);
            };
            
            ESP.restart();
        }

        if (storage.isConfigured()) {
          powerOn = storage.readBool(EEADDR_ENC_OUT_POS);
        } else {
          storage.writeBool(EEADDR_ENC_OUT_POS, false);
        }
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
      storage.writeBool(EEADDR_ENC_OUT_POS, true);
    }

    void cmd_poweroff () {
      powerOn = false;
      storage.writeBool(EEADDR_ENC_OUT_POS, false);
    }
};

#endif
