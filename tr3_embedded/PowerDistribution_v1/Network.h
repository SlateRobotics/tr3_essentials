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

    char res_c;
    int16_t res_len = 0;
    
    uint16_t packetSize;
    uint8_t packet[16384];
    NetworkPacket networkPacket;

    void connectWifi() {
      while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi..");
        WiFi.begin(ssid, pass);
        delay(1000);
      }
      
      Serial.println("WiFi connected");
    }

    void connectServer() {
      while (!client.connect(host, port)) {
        Serial.println("Server connection failed");
        delay(1000);
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
      static char stateStr[8];
      dtostrf(state->position, 6, 4, stateStr);
      
      sprintf(req, "%s:%s;\r\n", actuatorId, stateStr);
      client.print(req);
    }

    void response () {
      res_len = strlen(res);
      
      bool noCommand = (res_len < 10);// || strstr(res, "nc;"));
      if (noCommand == true) {
        return;
      }

      for (uint16_t i = 0; i < res_len; i++) {
        res_c = res[i];
        switch (res_c) {
          case ':':
            bufIdx = 0;
            break;
          case ';':
            if (bufIdx == 0) {
              buf[bufIdx++] = res_c;
              break;
            }
            
            networkPacket = NetworkPacket(packet, packetSize);
            controller->parseCmd(networkPacket);
            packetIdx = 0;
            bufIdx = 0;
            res[0] = '\0';
            break;
          case ',':
            buf[bufIdx] = '\0';
            bufIdx = 0;
            if (packetIdx == 2) {
              packetSize = atoi(buf);
            }
            packet[packetIdx++] = atoi(buf);
            break;
          default:
            buf[bufIdx++] = res_c;
        }
      }
    }
};

#endif
