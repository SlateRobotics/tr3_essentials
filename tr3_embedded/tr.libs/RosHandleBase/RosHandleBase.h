#ifndef ROS_HANDLE_BASE_H
#define ROS_HANDLE_BASE_H

#include <Update.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt32.h>
#include <ESP32Hardware.h>
#include <tcpip_adapter.h>

#include "Config.h"
#include "RosHandle.h"
#include "RosHandleEvents.h"
#include "Timer.h"

#define RT_IP               "/tr3/" NODE_ID_STR "/ip"
#define RT_VERSION          "/tr3/" NODE_ID_STR "/version"
#define RT_LOG              "/tr3/" NODE_ID_STR "/log"
#define RT_OTA_START        "/tr3/" NODE_ID_STR "/ota/start"
#define RT_OTA_DATA         "/tr3/" NODE_ID_STR "/ota/data"
#define RT_OTA_END          "/tr3/" NODE_ID_STR "/ota/end"

// Generic handle for communicating with ROS. Contains basic interfaces
// all ESP32-ROS nodes utilize as well as defining connection recovery behavior
namespace RosHandleBase {
    ros::NodeHandle_<ESP32Hardware>* nh;
    Timer configTimer(0.5); // hz

    tcpip_adapter_ip_info_t ipInfo;

    long conn_failure_delay = 200;
    int conn_failure_count = 0;
    const int max_conn_failure_count = 5;

    uint32_t bytesWritten = 0;
    uint32_t fileSize = 0;
    long uploadStart = 0;

    std_msgs::String msg_ip_addr;
    std_msgs::String msg_version;

    ros::Publisher pub_ip(RT_IP, &msg_ip_addr);
    ros::Publisher pub_version(RT_VERSION, &msg_version);

    void sub_cb_ota_start (const std_msgs::UInt32 &msg) {
        Serial.println("Firmware upload started...");
        size_t packet_len = msg.data;
        Update.begin(packet_len);
        
        bytesWritten = 0;
        fileSize = packet_len;
        uploadStart = millis();
    }
  
    void sub_cb_ota_data (const std_msgs::UInt8MultiArray &msg) {
        size_t packet_len = msg.data_length;
        bytesWritten += packet_len;
        Update.write(msg.data, packet_len);

        long duration = millis() - uploadStart;
        Serial.print("Uploading - ");
        Serial.print(duration);
        Serial.print(" - ");
        Serial.print((int)bytesWritten);
        Serial.print(" of ");
        Serial.println((int)fileSize);
    }

    void sub_cb_ota_end (const std_msgs::Bool &msg) {
        bool result = Update.end(true);

        if (result == true) {
            Serial.println("Firmware upload successful. Restarting...");
            ESP.restart();
        } else {
            Serial.println("Firmware upload failed. Please try again...");
        }
    }

    ros::Subscriber<std_msgs::UInt32> sub_ota_start(RT_OTA_START, &sub_cb_ota_start);
    ros::Subscriber<std_msgs::UInt8MultiArray> sub_ota_data(RT_OTA_DATA, &sub_cb_ota_data);
    ros::Subscriber<std_msgs::Bool> sub_ota_end(RT_OTA_END, &sub_cb_ota_end);

    bool waitForConnect (long timems) {
        long later = millis() + timems;
        while (later > millis()) {
            nh->spinOnce();
            if (nh->connected()) {
                return true;
            }
            delay(1);
        }
        return false;
    }

    void setup (ros::NodeHandle_<ESP32Hardware>* _nh) {
        nh = _nh;

        // subscribers
        nh->subscribe(sub_ota_start);
        nh->subscribe(sub_ota_data);
        nh->subscribe(sub_ota_end);

        // publishers
        nh->advertise(pub_ip);
        nh->advertise(pub_version);

        // config & connect
        nh->initNode();
        nh->setSpinTimeout(50);

        Serial.println("Establishing connection...");
        bool result = waitForConnect(2000);
        if (result == true) {
            Serial.println("Successfully connected to ROS master.");
        }
    }
    
    void step () {
        if (configTimer.ready()) {
            char ip_str[64];
            tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
            sprintf(ip_str, "%d.%d.%d.%d", IP2STR(&ipInfo.ip));

            msg_ip_addr.data = ip_str;
            pub_ip.publish(&msg_ip_addr);

            msg_version.data = NODE_VERSION;
            pub_version.publish(&msg_version);
        }
    }

    void connectRecovery () {
        waitForConnect(conn_failure_delay);
        while (!nh->connected()) {
            if (conn_failure_count < max_conn_failure_count) {
                RosHandleEvents::handle_ConnectionFailure();
                conn_failure_count++;
                Serial.print("Connection failed. Attempting to re-establish... [");
                Serial.print(conn_failure_count);
                Serial.print(" of ");
                Serial.print(max_conn_failure_count);
                Serial.println("]");
                waitForConnect(conn_failure_delay);
            } else {
                Serial.println("Max try limit reached. Restarting...");
                ESP.restart();
            }
        }

        RosHandleEvents::handle_ConnectionRecovery();
        Serial.println("Succesfully recovered connection");
        conn_failure_count = 0;
    }
}

#endif