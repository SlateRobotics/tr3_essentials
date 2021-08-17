#ifndef ROS_ESP32_HARDWARE_H_
#define ROS_ESP32_HARDWARE_H_

#include "Config.h"

extern "C" {
    #include "sdkconfig.h"
    #include "stdio.h"
    #include "esp_err.h"
    #include "esp_timer.h"
    #include <driver/uart.h>
    #include "esp_ros_wifi.h"
}

#define UART_PORT           UART_NUM_0
#define UART_TX_PIN         GPIO_NUM_1
#define UART_RX_PIN         GPIO_NUM_3

class ESP32Hardware {
    protected:
        uint8_t rx_buf[8192];

    public:
        ESP32Hardware() { }

        // Initialization code for ESP32
        void init() {
            esp_ros_wifi_init(CONFIG_ROSSERVER_SSID, CONFIG_ROSSERVER_PASS);
            ros_tcp_connect(CONFIG_ROSSERVER_IP, CONFIG_ROSSERVER_PORT);
        }

        // read a byte from the serial port. -1 = failure
        int read() {
            int read_len = ros_tcp_read(rx_buf, 1);
            if (read_len == 1) {
                return rx_buf[0];
            } else {
                return -1;
            }
        }

        // write data to the connection to ROS
        int write(uint8_t* data, int length) {
            return ros_tcp_send(data, length);
        }

        // returns milliseconds since start of program
        unsigned long time() {
            return esp_timer_get_time() / 1000;
        }
};
#endif
