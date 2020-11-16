#include <Arduino.h>

#include "Config.h"
#include "Controller.h"
#include "RosHandle.h"

Controller controller;

void setup() {
  Serial.begin(115200);
  
  Serial.print("Power Distribution: ");
  Serial.print(NODE_ID);
  Serial.print(", ");
  Serial.println(NODE_VERSION);

  controller.setUp();

  RosHandle::setup(&controller);
}

void loop() {
  RosHandle::step();
  controller.step();
}
