#include "custom.h"

#define MEMS_DISABLED 0
#define MEMS_ACC 1
#define MEMS_9DOF 2
#define MEMS_DMP 3

#define STORAGE_NONE 0
#define STORAGE_SD 1
#define STORAGE_SPIFFS 2

/**************************************
* Data logging
**************************************/
#ifndef HAVE_CONFIG
// enable(1)/disable(0) serial data output
#define ENABLE_SERIAL_OUT CUSTOM_ENABLE_SERIAL_OUT
// specify storage type
#define STORAGE STORAGE_SD
#endif

/**************************************
* WIFI and HTTP server
**************************************/
#ifndef HAVE_CONFIG
#define ENABLE_HTTPD CUSTOM_ENABLE_HTTPD
#define ENABLE_WIFI_AP 0
#define ENABLE_WIFI_STATION CUSTOM_ENABLE_WIFI_STATION
#define WIFI_AP_SSID "DATALOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"
#define WIFI_SSID CUSTOM_WIFI_SSID
#define WIFI_PASSWORD CUSTOM_WIFI_PASSWORD
#endif

#define WIFI_JOIN_TIMEOUT 30000
#define ENABLE_NMEA_SERVER 0
#define NMEA_TCP_PORT 4000

/**************************************
* Hardware setup
**************************************/
#ifndef HAVE_CONFIG
// enable(1)/disable(0) OBD-II reading
#define USE_OBD CUSTOM_USE_OBD
// GNSS option: 0:disable 1:internal 2:external 3:SIM5360/7600 4:SIM7070
#define USE_GNSS CUSTOM_USE_GNSS
// enable(1)/disable(0) MEMS motion sensor
#define USE_MEMS 1
#endif

// enable(1)/disable(0) BLE SPP server (for Freematics Controller App).
#define ENABLE_BLE 0

// GPS parameters
#define GPS_SERIAL_BAUDRATE 115200L
// motion detection
#define WAKEUP_MOTION_THRESHOLD 0.03 /* G */
// minimum loop time
#define MIN_LOOP_TIME CUSTOM_MIN_LOOP_TIME /* ms */ 