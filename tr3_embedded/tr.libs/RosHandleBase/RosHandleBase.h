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

#define RT_IP               "/tr3/" NODE_ID "/ip"
#define RT_VERSION          "/tr3/" NODE_ID "/version"
#define RT_LOG              "/tr3/" NODE_ID "/log"
#define RT_OTA_START        "/tr3/" NODE_ID "/ota/start"
#define RT_OTA_DATA         "/tr3/" NODE_ID "/ota/data"
#define RT_OTA_END          "/tr3/" NODE_ID "/ota/end"

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

    char log_data[256];

    std_msgs::String msg_ip_addr;
    std_msgs::String msg_version;
    std_msgs::String msg_log;

    ros::Publisher pub_ip(RT_IP, &msg_ip_addr);
    ros::Publisher pub_log(RT_LOG, &msg_log);
    ros::Publisher pub_version(RT_VERSION, &msg_version);

    void print (const char* val) {
        strcat(log_data, val);
    }

    void print (const double val) {
        char v[16];
        sprintf(v, "%f", val);
        print(v);
    }

    void print (const long val) {
        char v[16];
        sprintf(v, "%d", val);
        print(v);
    }

    void print (const int val) {
        char v[16];
        sprintf(v, "%d", val);
        print(v);
    }

    void println () {
        msg_log.data = log_data;
        pub_log.publish(&msg_log);
        Serial.println(log_data);
        memset(log_data, 0, sizeof log_data);
    }

    void println (const char* val) {
        print(val);
        println();
    }

    void println (const double val) {
        print(val);
        println();
    }

    void println (const long val) {
        print(val);
        println();
    }

    void println (const int val) {
        print(val);
        println();
    }

    void sub_cb_ota_start (const std_msgs::UInt32 &msg) {
        println("Firmware upload started...");
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
        print("Uploading - ");
        print(duration);
        print(" - ");
        print((int)bytesWritten);
        print(" of ");
        println((int)fileSize);
    }

    void sub_cb_ota_end (const std_msgs::Bool &msg) {
        bool result = Update.end(true);

        if (result == true) {
            println("Firmware upload successful. Restarting...");
            ESP.restart();
        } else {
            println("Firmware upload failed. Please try again...");
        }
    }

    ros::Subscriber<std_msgs::UInt32> sub_ota_start(RT_OTA_START, &sub_cb_ota_start);
    ros::Subscriber<std_msgs::UInt8MultiArray> sub_ota_data(RT_OTA_DATA, &sub_cb_ota_data);
    ros::Subscriber<std_msgs::Bool> sub_ota_end(RT_OTA_END, &sub_cb_ota_end);

    void setup (ros::NodeHandle_<ESP32Hardware>* _nh) {
        nh = _nh;
        
        // subscribers
        nh->subscribe(sub_ota_start);
        nh->subscribe(sub_ota_data);
        nh->subscribe(sub_ota_end);

        // publishers
        nh->advertise(pub_ip);
        nh->advertise(pub_log);
        nh->advertise(pub_version);
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
        while (!nh->connected()) {
            if (conn_failure_count < max_conn_failure_count) {
                RosHandleEvents::handle_ConnectionFailure();
                conn_failure_count++;
                Serial.print("Failed to connect... trying again in ");
                Serial.print(conn_failure_delay);
                Serial.print(" ms... [Attempt ");
                Serial.print(conn_failure_count);
                Serial.print(" of ");
                Serial.print(max_conn_failure_count);
                Serial.println("]");
                delay(conn_failure_delay);
            } else {
                Serial.println("Max try limit reached. Restarting...");
                ESP.restart();
            }
            nh->spinOnce();
        }

        RosHandleEvents::handle_ConnectionRecovery();
        println("Succesfully recovered connection");
        conn_failure_count = 0;
    }
}

#endif