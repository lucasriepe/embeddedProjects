#ifndef WEATHER_STATION_UI_H
#define WEATHER_STATION_UI_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize the weather station UI
 * 
 * This function initializes the complete weather station user interface,
 * including WiFi connection, time synchronization, DHT22 sensor, and LVGL display.
 */
void weather_station_ui_init(void);

/**
 * Cleanup the weather station UI
 * Stops timers and cleans up HTTP client resources
 */
void weather_station_ui_cleanup(void);

#endif // WEATHER_STATION_UI_H