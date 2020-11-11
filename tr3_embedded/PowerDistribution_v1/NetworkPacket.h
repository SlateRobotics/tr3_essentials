#ifndef NETWORKPACKET_H
#define NETWORKPACKET_H

class NetworkPacket {
  private:
  
  public:
    uint8_t msgId = 0;
    uint8_t address = 0;
    uint16_t length = 0;
    uint8_t command = 0;
    uint8_t* parameters;

    NetworkPacket () { }
    
    NetworkPacket (uint8_t* packet, uint16_t packetSize = 0) {
      msgId = packet[0];
      address = packet[1];
      length = packet[2];
      command = packet[3];
        
      if (packetSize > 0) {
        length = packetSize;
      }

      // if it's update firmware, ignore length max value
      if (command == CMD_UPDATE_FIRMWARE) {
        
      } else  {
        if (length - 4 > 256) {
          length = 256;
        }
      }

      parameters = (uint8_t*) packet + 4;
    }

    String toString() {
      String result = "";
      result += msgId;
      result += ",";
      result += address;
      result += ",";
      result += length;
      result += ",";
      result += command;
      result += ",";

      for (int i = 4; i < length; i++) {
        result += parameters[i - 4];
        result += ",";
      }
      
      result += ";";
      return result;
    }
};

#endif
