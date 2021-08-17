#pragma once

void esp_ros_wifi_init(const char* ssid, const char* pass);

void ros_tcp_connect(const char* host_ip, int port_num);

int ros_tcp_send(uint8_t* data, int length);

int ros_tcp_read(uint8_t* buf, int length);
