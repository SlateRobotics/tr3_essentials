#include "Config.h"

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <ESP32Hardware.h>

ros::NodeHandle_<ESP32Hardware> nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[64] = "hello world:";

void setup () {
    Serial.begin(115200);
    Serial.println("Begin");

    nh.initNode();
    nh.advertise(chatter);
}

void loop () {
    String s = String("hey there: ");
    s.concat(millis());
    s.toCharArray(hello, 64);

    str_msg.data = hello;
    chatter.publish(&str_msg);
    nh.spinOnce();

    delay(10);
}