#include <Arduino.h>

#include "Config.h"
#include "Controller.h"
#include "RosHandle.h"

Controller controller;

long start;

void setup() {
  Serial.begin(115200);

  Serial.print("Actuator: ");
  Serial.print(NODE_ID_STR);
  Serial.print(", ");
  Serial.println(NODE_VERSION);

  controller.requireImu = false;
  controller.setUp();
  controller.step();
  
  #if NODE_INIT_CALIBRATION == 1
    controller.storage.reset();
  #else
    RosHandle::setup(&controller);
  #endif

  start = millis();
}

void loop() {
  #if NODE_INIT_CALIBRATION == 1
    controller.step();
  #else
    RosHandle::step();
    controller.step();
  #endif
}
