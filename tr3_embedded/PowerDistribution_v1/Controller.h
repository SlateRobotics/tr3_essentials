#ifndef CONTROLLER_H
#define CONTROLLER_H

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

#define CMD_UPDATE_FIRMWARE_BEGIN 0x01
#define CMD_UPDATE_FIRMWARE 0x02
#define CMD_UPDATE_FIRMWARE_END 0x03
#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_STOP_RELEASE 0x15
#define CMD_STOP_EMERGENCY 0x16
#define CMD_FLIP_MOTOR 0x17

#define MODE_UPDATE_FIRMWARE 0x01
#define MODE_STOP 0x0F
#define MODE_SERVO 0x10
#define MODE_BACKDRIVE 0x11
#define MODE_ROTATE 0x12

#define PIN_RELAY 14

#include <ESPmDNS.h>
#include <Update.h>

#include "ControllerState.h"
#include "LED.h"
#include "NetworkPacket.h"
#include "Timer.h"

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
      int param = packet.parameters[0] + packet.parameters[1] * 256;
      double pos = param / 65535.0 * TAU;
      if (pos == 0) {
        state.position = LOW;
      } else {
        state.position = HIGH;
      }
    }
};

#endif
