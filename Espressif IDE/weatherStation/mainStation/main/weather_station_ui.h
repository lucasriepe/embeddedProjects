#ifndef WEATHER_STATION_UI_H
#define WEATHER_STATION_UI_H

#include "esp_now_comm.h"

/**
 * @brief Initialize the weather station UI
 * 
 * This function initializes the complete weather station user interface,
 * including WiFi connection, time synchronization, DHT22 sensor, ESP-NOW communication, and LVGL display.
 */
void weather_station_ui_init(void);

/**
 * @brief Register a remote ESP-NOW sensor
 * 
 * @param mac_addr MAC address of the remote sensor (6 bytes)
 * @param sensor_id Unique sensor ID
 * @param name Human readable name for the sensor
 * @return esp_err_t ESP_OK on success
 */
esp_err_t weather_station_register_remote_sensor(const uint8_t *mac_addr, uint8_t sensor_id, const char *name);

/**
 * @brief Get the number of active remote sensors
 * 
 * @return uint8_t Number of active remote sensors
 */
uint8_t weather_station_get_active_remote_sensors(void);

#endif // WEATHER_STATION_UI_H