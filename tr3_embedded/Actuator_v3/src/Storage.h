#ifndef XT_STORAGE_H
#define XT_STORAGE_H

#define EEPROM_SIZE 256

#define EEADDR_EE_SET_1 0 // uint8, 1 byte
#define EEADDR_EE_SET_2 1 // uint8, 1 byte
#define EEADDR_ENC_OUT_OFFSET 2 // uint16, 2 bytes
#define EEADDR_ENC_TRQ_OFFSET 4 // uint16, 2 bytes
#define EEADDR_MTR_FLIP 6 // bool, 1 byte
#define EEADDR_SEA_SPRING_RATE 27 // float, 4 bytes
#define EEADDR_ENC_OUT_UP 9 // bool, 1 byte
#define EEADDR_ENC_OUT_POS 10 // double, 8 bytes
#define EEADDR_ENC_OUT_LAP 55 // double, 8 bytes
#define EEADDR_ENC_TRQ_UP 18 // bool, 1 byte
#define EEADDR_ENC_TRQ_POS 19 // double, 8 bytes
#define EEADDR_ENC_TRQ_LAP 63 // double, 8 bytes
#define EEADDR_PID_POS_P 71 // float, 4 bytes
#define EEADDR_PID_VEL_P 75 // float, 4 bytes
#define EEADDR_PID_VEL_I 79 // float, 4 bytes

#define EE_SET_1 0x51
#define EE_SET_2 0x22

#include "EEPROM.h"

class Storage {
  private:

  public:
    bool begin() {
      return EEPROM.begin(EEPROM_SIZE);
    }

    void configure () {
      writeUInt8(EEADDR_EE_SET_1, EE_SET_1);
      writeUInt8(EEADDR_EE_SET_2, EE_SET_2);
      writeUInt16(EEADDR_ENC_OUT_OFFSET, 0);
      writeUInt16(EEADDR_ENC_TRQ_OFFSET, 0);
      writeBool(EEADDR_MTR_FLIP, false);
      writeUInt16(EEADDR_SEA_SPRING_RATE, 75);
      writeBool(EEADDR_ENC_OUT_UP, false);
      writeFloat(EEADDR_ENC_OUT_POS, 0.0);
      writeBool(EEADDR_ENC_TRQ_UP, false);
      writeFloat(EEADDR_ENC_TRQ_POS, 0.0);
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
