#include "Config.h"
#include "Controller.h"
#include "Network.h"
#include "Timer.h"

Networking networking;
Controller controller;
Timer timer(20); // hz

void setup() {
  Serial.begin(115200);

  Serial.print("Setting up ");
  Serial.println(ACTUATOR_ID);

  controller.requireImu = false;
  controller.setUp();

  networking.controller = &controller;
  networking.ssid = TR2_AN_SSID;
  networking.pass = TR2_AN_PASS;
  networking.connect();
}

void loop() { 
  if (timer.ready()) {
    ControllerState* state = controller.getState();
    networking.step(ACTUATOR_ID, state);
  }

  controller.step();
}
