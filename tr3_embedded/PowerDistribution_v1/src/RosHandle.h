#ifndef ROS_HANDLE_H
#define ROS_HANDLE_H

#include "Config.h"
#include "Controller.h"

#include <ros.h>
#include <string.h>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <ESP32Hardware.h>
#include <tcpip_adapter.h>

#include "Controller.h"
#include "RosHandleBase.h"
#include "Timer.h"

namespace RosHandle {
  ros::NodeHandle_<ESP32Hardware> nh;
  Controller *controller;

  Timer configTimer(1); // hz
  Timer nhTimer(200); // hz
  
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

  std_msgs::Bool state;

  ros::Publisher pub_state(RT_STATE, &state);

  void setup(Controller* c) {
    controller = c;

    // subscribers
    nh.subscribe(sub_poweron);
    nh.subscribe(sub_poweroff);

    // publishers
    nh.advertise(pub_state);

    RosHandleBase::setup(&nh);
  }

  void step() {
    RosHandleBase::step();

    std_msgs::Bool p;
    p.data = controller->poweredOn();
    pub_state.publish(&p);
    
    // the issue is that nh.connected()
    // returns true even if no connection
    // update node_handle.h to mark as unconnected
    // if write() or read() error on ESP32Hardware.h

    // also, result from spinOnce() returns OK
    // when an error occurs here as well...

    if (nh.connected()) {
      int result = nh.spinOnce();
      if (result != ros::SPIN_OK) {
        RosHandleBase::connectRecovery();
      }
    } else {
      RosHandleBase::connectRecovery();
    }
  }
}


#endif