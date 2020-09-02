#ifndef NETWORKING_H
#define NETWORKING_H

#include "Controller.h"
#include "ControllerState.h"
#include "NetworkPacket.h"
#include "WiFi.h"

class Networking {
  private:
    WiFiClient client;
    char req[256];
    char res[16384];
    char buf[256];
    
    uint16_t bufIdx = 0;
    uint16_t packetIdx = 0;
    char c;
    
    uint8_t packet[4096];
    NetworkPacket networkPacket;

    void connectWifi() {
      WiFi.begin(ssid, pass);
    
      while (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        Serial.println("Connecting to WiFi..");
      }
      
      Serial.println("WiFi connected");
    }

    void connectServer() {
      if (!client.connect(host, port)) {
        Serial.println("Server connection failed");
        delay(500);
        connectServer();
      }
      
      Serial.println("Server connected");
    }
  
  public:
    Controller* controller;
    
    char* host = "192.168.4.1";
    uint16_t port = 1738;
    char* ssid;
    char* pass;

    void connect() {
      connectWifi();
      connectServer();
    }

    void step (char* actuatorId, ControllerState* state) {
      request(actuatorId, state);
      client.readStringUntil('\r').toCharArray(res, 16384);
      response();
    }

    void request (char* actuatorId, ControllerState* state) {
      static char stateStr[64];
      state->toString().toCharArray(stateStr, 64);
      //dtostrf(state->position, 6, 4, stateStr);
      
      sprintf(req, "%s:%s;\r\n", actuatorId, stateStr);
      client.print(req);
    }

    void response () {
      // no command
      if (strstr(res, "nc;") || strlen(res) == 0) {
        return;
      }
      
      uint16_t _len;
      //Serial.println(millis());
      for (int i = 0; i < strlen(res); i++) {
        c = res[i];
        if (c == ':') {
          bufIdx = 0;
        } else if (c == ';' && packetIdx > 0) {
          //Serial.println(millis());
          networkPacket = NetworkPacket(packet, _len);
          controller->parseCmd(networkPacket);
          packetIdx = 0;
          bufIdx = 0;
        } else if (c != ',') {
          buf[bufIdx++] = c;
        } else {
          buf[bufIdx] = '\0';
          bufIdx = 0;
          if (packetIdx == 2) {
            _len = atoi(buf);
          }
          packet[packetIdx++] = atoi(buf);
        }
      }
    }
};

#endif
