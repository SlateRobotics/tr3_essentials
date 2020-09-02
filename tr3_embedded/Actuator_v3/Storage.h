#ifndef XT_STORAGE_H
#define XT_STORAGE_H

#define EEPROM_SIZE 256

#define EEADDR_EE_SET_1 0
#define EEADDR_EE_SET_2 1
#define EEADDR_ENC_O_OFFSET 2
#define EEADDR_ENC_D_OFFSET 4
#define EEADDR_MTR_FLIP 6
#define EEADDR_SEA_SPRING_RATE 7
#define EEADDR_ENC_O_UP 9
#define EEADDR_ENC_O_POS 10
#define EEADDR_ENC_O_LAP 55
#define EEADDR_ENC_D_UP 18
#define EEADDR_ENC_D_POS 19
#define EEADDR_ENC_D_LAP 63
#define EEADDR_REG_C1_1 27
#define EEADDR_REG_C1_2 31
#define EEADDR_REG_C1_3 35
#define EEADDR_REG_C2_1 39
#define EEADDR_REG_C2_2 43
#define EEADDR_REG_C2_3 47
#define EEADDR_REG_OFFSET 51
#define EEADDR_PID_P 71
#define EEADDR_PID_I 75
#define EEADDR_PID_D 79

#define EE_SET_1 0x51
#define EE_SET_2 0x22

#include "EEPROM.h"

class Storage {
  private:

  public:
    bool setUp() {
      return EEPROM.begin(EEPROM_SIZE);
    }

    void configure () {
      writeUInt8(EEADDR_EE_SET_1, EE_SET_1);
      writeUInt8(EEADDR_EE_SET_2, EE_SET_2);
      writeUInt16(EEADDR_ENC_O_OFFSET, 0);
      writeUInt16(EEADDR_ENC_D_OFFSET, 0);
      writeBool(EEADDR_MTR_FLIP, false);
      writeUInt16(EEADDR_SEA_SPRING_RATE, 882);
      writeBool(EEADDR_ENC_O_UP, false);
      writeFloat(EEADDR_ENC_O_POS, 0.0);
      writeBool(EEADDR_ENC_D_UP, false);
      writeFloat(EEADDR_ENC_D_POS, 0.0);
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
