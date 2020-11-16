#ifndef CONFIG_H
#define CONFIG_H

// MATH HELPERS
#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

// NODE DETAILS
#define NODE_ID "p0"
#define NODE_VERSION "v3.0.0"

// ROSSERIAL CONFIG
#define CONFIG_ROSSERIAL_OVER_WIFI 1
#define CONFIG_ROSSERVER_IP "192.168.1.5"
#define CONFIG_ROSSERVER_PORT 11411
#define CONFIG_ROSSERVER_AP "TR_AN_J03NXO"
#define CONFIG_ROSSERVER_PASS "narrowroad512"

// ROSTOPICS USED BY NODE
#define RT_STATE            "/tr3/power/state"
#define RT_POWERON          "/tr3/power/on"
#define RT_POWEROFF         "/tr3/power/off"

// PINS USED BY NODE
#define PIN_RELAY 14
#define LED_DATA_PIN 25

#endif
