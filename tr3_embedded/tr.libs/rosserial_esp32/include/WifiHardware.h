#ifndef WIFI_HARDWARE_H
#define WIFI_HARDWARE_H

#include "Arduino.h"

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      // do your initialization here. this probably includes TCP server/client setup
      /*Serial.printf("WiFiHardware: init, hostname = %s, port = %d\r\n", "tr2-desktop", 11311);
      while (! client.connect(hostNameComputer, port)) {
        Serial.printf("Waiting for connection\r\n");
        delay(500);
      }*/
    }
    // read a byte from the serial port. -1 = failure
    int read() {
      // implement this method so that it reads a byte from the TCP connection and returns it
      // you may return -1 is there is an error; for example if the TCP connection is not open
      //return client.read(); //will return -1 when it will works
    }
    // write data to the connection to ROS
    void write(uint8_t* data, int length) {
      // implement this so that it takes the arguments and writes or prints them to the TCP connection
      /*for (int i = 0; i < length; i++) {
        client.write(data[i]);
      }*/
    }
    // returns milliseconds since start of program
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

#endif