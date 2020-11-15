#include <Arduino.h>

#include "Config.h"
#include "Controller.h"
#include "RosHandle.h"

Controller controller;

void setup() {
  Serial.begin(115200);

  Serial.print("Actuator: ");
  Serial.print(ACTUATOR_ID);
  Serial.print(", ");
  Serial.println(ACTUATOR_VERSION);

  controller.requireImu = false;
  controller.setUp();

  RosHandle::setup(&controller);
}

void loop() {
  //RosHandle::state = controller.getState();
  //RosHandle::pid_pos = controller.pidPos.getGains();
  //RosHandle::pid_vel = controller.pidVel.getGains();
  //RosHandle::pid_trq = controller.pidTrq.getGains();

  RosHandle::step();
  controller.step();
}
