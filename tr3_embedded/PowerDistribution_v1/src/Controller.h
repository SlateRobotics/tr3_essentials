#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESPmDNS.h>
#include <Update.h>

#include "Config.h"
#include "ControllerState.h"
#include "LED.h"
#include "NetworkPacket.h"
#include "Timer.h"
#include "Utils.h"

int status;

class Controller {
  private:
    int mode = MODE_ROTATE;
    int modePrev = MODE_ROTATE;

    uint16_t updatePacksRecv = 0;
    
    ControllerState state;
    LED led;
    Timer imuTimer = Timer(5); // hz

    void computeState () {
      if (mode == CMD_UPDATE_FIRMWARE_BEGIN || mode == CMD_UPDATE_FIRMWARE) {
        state.position = updatePacksRecv;
        return;
      }
    }
  
  public:

    Controller () { }

    void setUp () {
      led.setUp();
      pinMode(PIN_RELAY, OUTPUT);
      led.white();
    }

    ControllerState* getState () {
      return &state;
    }

    /// ----------------------
    /// --- STEP FUNCTIONS ---
    /// ----------------------
    void step () {
      led.pulse(LED_CYAN);
      computeState();
      digitalWrite(PIN_RELAY, state.position);
    }

    /// ---------------------
    /// --- CMD FUNCTIONS ---
    /// ---------------------
    void parseCmd (NetworkPacket packet) {
      if (packet.command == CMD_UPDATE_FIRMWARE_BEGIN) {
        led.white();
        mode = MODE_UPDATE_FIRMWARE;
        Update.begin(UPDATE_SIZE_UNKNOWN);
        updatePacksRecv = 0;
      } else if (packet.command == CMD_UPDATE_FIRMWARE) {
        // doing it this way, we process small chunks of the firmware
        // instead of putting the entire prog into mem, then updating
        led.white();
        uint16_t update_size = packet.length - 4;
        Update.write(packet.parameters, update_size);
        updatePacksRecv = updatePacksRecv + 1;
      } else if (packet.command == CMD_UPDATE_FIRMWARE_END) {
        led.white();
        Update.end(true);
        ESP.restart();
      } else if (packet.command == CMD_SET_POS) {
        cmd_setPosition(packet);
      }
    }

    void cmd_setPosition (NetworkPacket packet) {
      float pos = Utils::bytesToFloat(packet.parameters);
      Serial.println(pos);
      if (abs(pos) < 0.1) {
        state.position = LOW;
      } else {
        state.position = HIGH;
      }
    }
};

#endif
