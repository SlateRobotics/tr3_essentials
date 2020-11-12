#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESPmDNS.h>
#include <Update.h>
#include "EEPROM.h"

#include "Defaults.h"
#include "ControllerState.h"
#include "LED.h"
#include "NetworkPacket.h"
#include "Timer.h"

int status;

class Controller {
  private:
    int mode = MODE_ROTATE;
    int modePrev = MODE_ROTATE;

    const static uint16_t readCellCount = 256;
    float readCell1[readCellCount];
    float readCell2[readCellCount];
    float readCell3[readCellCount];

    uint16_t updatePacksRecv = 0;
    
    ControllerState state;
    LED led;
    Timer imuTimer = Timer(5); // hz

    void computeState () {
      if (mode == CMD_UPDATE_FIRMWARE_BEGIN || mode == CMD_UPDATE_FIRMWARE) {
        state.position = updatePacksRecv;
        return;
      }
      
      float eCell1 = analogRead(PIN_CELL_1) / 4095.0 * 3.3;
      float eCell2 = analogRead(PIN_CELL_2) / 4095.0 * 3.3;
      float eCell3 = analogRead(PIN_CELL_3) / 4095.0 * 3.3;

      eCell1 = eCell1 + (0.13 * (1.0 * eCell1 + 1.0) / (eCell1 + sqrt(0.05*pow(eCell1 - 1.0, 10.0) + 1.0)));
      eCell2 = eCell2 + (0.13 * (1.0 * eCell2 + 1.0) / (eCell2 + sqrt(0.05*pow(eCell2 - 1.0, 10.0) + 1.0)));
      eCell3 = eCell3 + (0.13 * (1.0 * eCell3 + 1.0) / (eCell3 + sqrt(0.05*pow(eCell3 - 1.0, 10.0) + 1.0)));

      for (uint16_t i = readCellCount - 1; i > 0; i--) {
        readCell1[i] = readCell1[i - 1];
        readCell2[i] = readCell2[i - 1];
        readCell3[i] = readCell3[i - 1];
      }

      readCell1[0] = eCell1 * (VD_R1 + VD_R2) / VD_R2;
      readCell2[0] = eCell2 * (VD_R1 + VD_R2) / VD_R2;
      readCell3[0] = eCell3 * (VD_R1 + VD_R2) / VD_R2  ;

      float aCell1 = 0;
      float aCell2 = 0;
      float aCell3 = 0;

      for (uint16_t i = 0; i < readCellCount; i++) {
        aCell1 = aCell1 + readCell1[i];
        aCell2 = aCell2 + readCell2[i];
        aCell3 = aCell3 + readCell3[i];
      }
   
      /*state.vCell1 = aCell1 / (float)readCellCount;
      state.vCell2 = aCell2 / (float)readCellCount;
      state.vCell3 = aCell3 / (float)readCellCount;*/
      
      state.position = aCell3 / (float)readCellCount;

      /*state.vCell3 = state.vCell3 - state.vCell2;
      state.vCell2 = state.vCell2 - state.vCell1;*/
      
    }

    void setUpConfig () {
      
    }
  
  public:

    Controller () { }

    void setUp () {
      led.setUp();
      pinMode(PIN_CELL_1, INPUT);
      pinMode(PIN_CELL_2, INPUT);
      pinMode(PIN_CELL_3, INPUT);
      led.white();

      for (uint16_t i = 0; i < readCellCount; i++) {
        computeState();
      }
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
      }
    }
};

#endif
