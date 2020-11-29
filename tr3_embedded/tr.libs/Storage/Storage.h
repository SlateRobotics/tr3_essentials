#ifndef XT_STORAGE_H
#define XT_STORAGE_H

#define EEPROM_SIZE 256

// EEPROM ADDRESSES
#define EEADDR_EE_SET_1 0
#define EEADDR_EE_SET_2 1
#define EEADDR_ENC_OUT_OFFSET 2
#define EEADDR_ENC_TRQ_OFFSET 4
#define EEADDR_MTR_FLIP 6
#define EEADDR_SEA_SPRING_RATE 107
#define EEADDR_ENC_OUT_UP 9
#define EEADDR_ENC_OUT_POS 10
#define EEADDR_ENC_OUT_LAP 55
#define EEADDR_ENC_TRQ_UP 18
#define EEADDR_ENC_TRQ_LAP 63
#define EEADDR_PID_POS_P 71
#define EEADDR_PID_POS_I 87
#define EEADDR_PID_POS_D 91
#define EEADDR_PID_VEL_P 75
#define EEADDR_PID_VEL_I 79
#define EEADDR_PID_VEL_D 83
#define EEADDR_PID_TRQ_P 95
#define EEADDR_PID_TRQ_I 99
#define EEADDR_PID_TRQ_D 103
#define EEADDR_POSITION_MIN 111
#define EEADDR_POSITION_MAX 115
#define EEADDR_VELOCITY_MIN 119
#define EEADDR_VELOCITY_MAX 123
#define EEADDR_TORQUE_MIN 127
#define EEADDR_TORQUE_MAX 131

// DEFAULT PID GAINS
#define DEFAULT_PID_POS_P 5.000
#define DEFAULT_PID_POS_I 0.000
#define DEFAULT_PID_POS_D 0.000
#define DEFAULT_PID_VEL_P 0.500
#define DEFAULT_PID_VEL_I 1.000
#define DEFAULT_PID_VEL_D 0.000
#define DEFAULT_PID_TRQ_P 0.450
#define DEFAULT_PID_TRQ_I 0.050
#define DEFAULT_PID_TRQ_D 0.000

// DEFAULT LIMITS
#define DEFAULT_POSITION_MIN -6.28
#define DEFAULT_POSITION_MAX 6.28
#define DEFAULT_VELOCITY_MIN -0.942
#define DEFAULT_VELOCITY_MAX 0.942
#define DEFAULT_TORQUE_MIN -60.0
#define DEFAULT_TORQUE_MAX 60.0
#define DEFAULT_MOTOR_MIN -1.0
#define DEFAULT_MOTOR_MAX 1.0

#define EE_SET_1 0x51
#define EE_SET_2 0x22

#include "EEPROM.h"

class Storage {
  private:

  public:
    void begin () {
      if (!EEPROM.begin(EEPROM_SIZE)) {
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
    }

    void reset () {
      writeUInt8(EEADDR_EE_SET_1, EE_SET_1);
      writeUInt8(EEADDR_EE_SET_2, EE_SET_2);

      writeBool(EEADDR_MTR_FLIP, false);
      writeFloat(EEADDR_SEA_SPRING_RATE, -350);

      writeUInt16(EEADDR_ENC_OUT_OFFSET, 0);
      writeBool(EEADDR_ENC_OUT_UP, false);
      writeFloat(EEADDR_ENC_OUT_POS, 0);
      writeFloat(EEADDR_ENC_OUT_LAP, 0);

      writeUInt16(EEADDR_ENC_TRQ_OFFSET, 0);
      writeBool(EEADDR_ENC_TRQ_UP, false);
      writeFloat(EEADDR_ENC_TRQ_LAP, 0);

      writeFloat(EEADDR_PID_POS_P, DEFAULT_PID_POS_P);
      writeFloat(EEADDR_PID_POS_I, DEFAULT_PID_POS_I);
      writeFloat(EEADDR_PID_POS_D, DEFAULT_PID_POS_D);
      writeFloat(EEADDR_PID_VEL_P, DEFAULT_PID_VEL_P);
      writeFloat(EEADDR_PID_VEL_I, DEFAULT_PID_VEL_I);
      writeFloat(EEADDR_PID_VEL_D, DEFAULT_PID_VEL_D);
      writeFloat(EEADDR_PID_TRQ_P, DEFAULT_PID_TRQ_P);
      writeFloat(EEADDR_PID_TRQ_I, DEFAULT_PID_TRQ_I);
      writeFloat(EEADDR_PID_TRQ_D, DEFAULT_PID_TRQ_D);

      writeFloat(EEADDR_POSITION_MIN, DEFAULT_POSITION_MIN);
      writeFloat(EEADDR_POSITION_MAX, DEFAULT_POSITION_MAX);
      writeFloat(EEADDR_VELOCITY_MIN, DEFAULT_VELOCITY_MIN);
      writeFloat(EEADDR_VELOCITY_MAX, DEFAULT_VELOCITY_MAX);
      writeFloat(EEADDR_TORQUE_MIN, DEFAULT_TORQUE_MIN);
      writeFloat(EEADDR_TORQUE_MAX, DEFAULT_TORQUE_MAX);

      commit();
    }

    void commit () {
      EEPROM.commit();
    }

    bool isConfigured () {
      uint8_t ee_set_1 = readUInt8(EEADDR_EE_SET_1);
      uint8_t ee_set_2 = readUInt8(EEADDR_EE_SET_2);
      return (ee_set_1 == EE_SET_1 && ee_set_2 == EE_SET_2);
    }

    // WRITE
    void writeUInt8(int addr, uint8_t value) {
      EEPROM.write(addr, value);
    }

    void writeUInt16 (int addr, uint16_t value) {
      EEPROM.put(addr, value);
    }

    void writeBool (int addr, bool value) {
      if (value == true) {
        writeUInt8(addr, 1);
      } else {
        writeUInt8(addr, 0);
      }
    }

    void writeFloat (int addr, float value) {
     byte* p = (byte*)(void*)&value;
     for (int i = 0; i < sizeof(value); i++) {
         EEPROM.write(addr + i, *p++);
     }
    }

    void writeDouble (int addr, double value) {
      EEPROM.put(addr, value);
    }

    // READ
    uint8_t readUInt8 (int addr) {
      return EEPROM.read(addr);
    }
    
    uint16_t readUInt16 (int addr) {
      uint16_t v;
      EEPROM.get(addr, v);
      return v;
    }

    bool readBool (int addr) {
      uint8_t v = readUInt8(addr);
      if (v > 0) {
        return true;
      } else {
        return false;
      }
    }

    float readFloat (int addr) {
      float value = 0.0;
       byte* p = (byte*)(void*)&value;
       for (int i = 0; i < sizeof(value); i++) {
           *p++ = EEPROM.read(addr + i);
       }
       return value;
    }

    double readDouble (int addr) {
      double v;
      EEPROM.get(addr, v);
      return v;
    }
};

#endif
