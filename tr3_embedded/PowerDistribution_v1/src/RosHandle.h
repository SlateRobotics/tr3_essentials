#ifndef ROS_HANDLE_H
#define ROS_HANDLE_H

#include "Config.h"
#include "Controller.h"

#include <ros.h>
#include <std_msgs/Bool.h>
#include <ESP32Hardware.h>

#include "Controller.h"
#include "Timer.h"

namespace RosHandle {
  ros::NodeHandle_<ESP32Hardware> nh;
  Controller *controller;

  int failures = 0;
  const int max_failures = 5;

  Timer pidTimer(5); // hz
  Timer nhTimer(200); // hz

  std_msgs::Bool state;

  ros::Publisher pub_state(RT_STATE, &state);
  
  void ros_callback_poweron (const std_msgs::Bool &msg) {
    if (msg.data == true) {
      controller->cmd_poweron();
    } else {
      controller->cmd_poweroff();
    }
  }
  
  void ros_callback_poweroff (const std_msgs::Bool &msg) {
    if (msg.data == true) {
      controller->cmd_poweroff();
    } else {
      controller->cmd_poweron();
    }
  }

  ros::Subscriber<std_msgs::Bool> sub_poweron(RT_POWERON, &ros_callback_poweron);
  ros::Subscriber<std_msgs::Bool> sub_poweroff(RT_POWEROFF, &ros_callback_poweroff);

  void connectRecovery () {
    while (!nh.connected()) {
      if (failures < max_failures) {
        failures++;
        //controller->cmd_poweroff();
        Serial.print("Failed to connect... trying again in 200ms... [Attempt ");
        Serial.print(failures);
        Serial.print(" of ");
        Serial.print(max_failures);
        Serial.println("]");
        delay(200);
      } else {
        Serial.println("Max try limit reached. Restarting...");
        ESP.restart();
        delay(5000);
      }
      nh.spinOnce();
    }

    Serial.println("Succesfully recovered connection");
    failures = 0;
    //controller->cmd_poweron();
  }

  void setup(Controller* c) {
    controller = c;

    nh.initNode();

    // subscribers
    nh.subscribe(sub_poweron);
    nh.subscribe(sub_poweroff);


    // publishers
    nh.advertise(pub_state);
  }

  void step() {
    if (nhTimer.ready()) {
      std_msgs::Bool p;
      p.data = controller->poweredOn();
      RosHandle::pub_state.publish(&p);

      // error when esp32 spins, then we start rosserial server
      // we need to figure out how to intelligently restart connection
      // if we fail to send/receive data
      int result = nh.spinOnce();
      if (result != ros::SPIN_OK || !nh.connected()) {
        connectRecovery();
      }
    }
  }
}

#endif