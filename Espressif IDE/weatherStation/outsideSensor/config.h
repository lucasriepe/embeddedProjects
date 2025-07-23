/*
 * Configuration file for ESP8266 Remote Sensor
 * 
 * Here different sensor configurations can be
 * quickly adjusted.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ========== SENSOR CONFIGURATION ==========
// Unique sensor ID (MUST be different for each sensor!)
#define SENSOR_ID 1

// Sensor name for debugging
#define SENSOR_NAME "Outside Sensor"

// ========== HARDWARE PINS ==========
#define DHT_PIN 2              // GPIO2 (D4 on NodeMCU)
#define DHT_TYPE DHT22         // DHT22 or DHT11

// ========== TIMING CONFIGURATION ==========
#define SEND_INTERVAL 30000    // Send interval in ms (30 seconds)
#define SENSOR_WARMUP 5000     // Sensor warmup time in ms

// ========== POWER MANAGEMENT ==========
#define DEEP_SLEEP_ENABLED false    // true = Enable Deep Sleep
#define DEEP_SLEEP_MINUTES 5        // Deep Sleep duration in minutes

// ========== NETWORK CONFIGURATION ==========
// MainStation MAC Address - MUST BE ADJUSTED!
// Get with mac_scanner.ino or from MainStation Serial output
#define MAIN_STATION_MAC {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}

// ========== DEBUGGING ==========
#define DEBUG_ENABLED true     // Serial debug output
#define SERIAL_BAUD 115200     // Serial baud rate

// ========== SENSOR LIMITS ==========
#define TEMP_MIN -40.0         // Minimum valid temperature
#define TEMP_MAX 80.0          // Maximum valid temperature
#define HUMIDITY_MIN 0.0       // Minimum valid humidity
#define HUMIDITY_MAX 100.0     // Maximum valid humidity

// ========== PREDEFINED CONFIGURATIONS ==========

// Configuration for indoor sensor
#ifdef INDOOR_SENSOR
    #undef SENSOR_ID
    #undef SENSOR_NAME
    #undef SEND_INTERVAL
    #define SENSOR_ID 1
    #define SENSOR_NAME "Indoor"
    #define SEND_INTERVAL 60000    // 1 minute
#endif

// Configuration for outdoor sensor
#ifdef OUTDOOR_SENSOR
    #undef SENSOR_ID
    #undef SENSOR_NAME
    #undef SEND_INTERVAL
    #define SENSOR_ID 2
    #define SENSOR_NAME "Outdoor"
    #define SEND_INTERVAL 30000    // 30 seconds
#endif

// Configuration for battery sensor (power saving mode)
#ifdef BATTERY_SENSOR
    #undef SENSOR_ID
    #undef SENSOR_NAME
    #undef SEND_INTERVAL
    #undef DEEP_SLEEP_ENABLED
    #undef DEEP_SLEEP_MINUTES
    #define SENSOR_ID 3
    #define SENSOR_NAME "Battery Sensor"
    #define SEND_INTERVAL 10000    // 10 seconds (before Deep Sleep)
    #define DEEP_SLEEP_ENABLED true
    #define DEEP_SLEEP_MINUTES 10  // 10 minutes Deep Sleep
#endif

// Configuration for greenhouse sensor
#ifdef GREENHOUSE_SENSOR
    #undef SENSOR_ID
    #undef SENSOR_NAME
    #undef SEND_INTERVAL
    #define SENSOR_ID 4
    #define SENSOR_NAME "Greenhouse"
    #define SEND_INTERVAL 15000    // 15 seconds
#endif

#endif // CONFIG_H

/*
 * USAGE:
 * 
 * 1. Standard configuration:
 *    Simply include config.h and adjust values above
 * 
 * 2. Predefined configuration:
 *    #define OUTDOOR_SENSOR before #include "config.h"
 * 
 * 3. Example for multiple sensors:
 *    - Sensor 1: #define INDOOR_SENSOR
 *    - Sensor 2: #define OUTDOOR_SENSOR  
 *    - Sensor 3: #define BATTERY_SENSOR
 *    - Sensor 4: #define GREENHOUSE_SENSOR
 */