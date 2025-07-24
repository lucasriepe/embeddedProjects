/*
 * ESP-NOW Communication Module
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for esp_err_t
#ifndef ESP_ERR_T_DEFINED
#define ESP_ERR_T_DEFINED
typedef int esp_err_t;
#endif

// ESP error codes (commonly used ones)
#ifndef ESP_OK
#define ESP_OK          0       /*!< esp_err_t value indicating success (no error) */
#define ESP_FAIL        -1      /*!< Generic esp_err_t code indicating failure */
#define ESP_ERR_NO_MEM          0x101   /*!< Out of memory */
#define ESP_ERR_INVALID_ARG     0x102   /*!< Invalid argument */
#define ESP_ERR_INVALID_STATE   0x103   /*!< Invalid state */
#define ESP_ERR_INVALID_SIZE    0x104   /*!< Invalid size */
#define ESP_ERR_NOT_FOUND       0x105   /*!< Requested resource not found */
#define ESP_ERR_NOT_SUPPORTED   0x106   /*!< Operation or feature not supported */
#define ESP_ERR_TIMEOUT         0x107   /*!< Operation timed out */
#endif

// Maximum number of remote sensors that can be registered
#define ESP_NOW_MAX_REMOTE_SENSORS 10

// ESP-NOW data packet structure for sensor data
typedef struct {
    uint8_t sensor_id;          // Unique sensor identifier
    float temperature;          // Temperature in Celsius
    float humidity;             // Humidity in percentage
    uint32_t timestamp;         // Timestamp of measurement
    uint16_t battery_voltage;   // Battery voltage in mV (optional, 0 for mains-powered)
    bool valid;                 // Data validity flag
} esp_now_sensor_data_t;

// Remote sensor information
typedef struct {
    uint8_t mac_addr[6];        // MAC address of remote sensor
    uint8_t sensor_id;          // Sensor ID
    char name[32];              // Human readable name
    esp_now_sensor_data_t last_data; // Last received data
    uint32_t last_seen;         // Last time data was received
    bool is_active;             // Is sensor currently active
} esp_now_remote_sensor_t;

// Callback function type for received sensor data
typedef void (*esp_now_data_callback_t)(const esp_now_sensor_data_t *data, const uint8_t *mac_addr);

/**
 * @brief Initialize ESP-NOW communication
 * 
 * @param data_callback Callback function to handle received sensor data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_now_comm_init(esp_now_data_callback_t data_callback);

/**
 * @brief Deinitialize ESP-NOW communication
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_now_comm_deinit(void);

/**
 * @brief Register a remote sensor
 * 
 * @param mac_addr MAC address of the remote sensor
 * @param sensor_id Unique sensor ID
 * @param name Human readable name for the sensor
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_now_register_sensor(const uint8_t *mac_addr, uint8_t sensor_id, const char *name);

/**
 * @brief Get information about a registered sensor
 * 
 * @param sensor_id Sensor ID to query
 * @param sensor_info Pointer to store sensor information
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if sensor not found
 */
esp_err_t esp_now_get_sensor_info(uint8_t sensor_id, esp_now_remote_sensor_t *sensor_info);

/**
 * @brief Get list of all registered sensors
 * 
 * @param sensors Array to store sensor information
 * @param max_sensors Maximum number of sensors to return
 * @param count Pointer to store actual number of sensors returned
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_now_get_all_sensors(esp_now_remote_sensor_t *sensors, uint8_t max_sensors, uint8_t *count);

/**
 * @brief Check if a sensor is considered online (received data recently)
 * 
 * @param sensor_id Sensor ID to check
 * @param timeout_ms Timeout in milliseconds to consider sensor offline
 * @return true if sensor is online, false otherwise
 */
bool esp_now_is_sensor_online(uint8_t sensor_id, uint32_t timeout_ms);

/**
 * @brief Send a command to a remote sensor (optional feature)
 * 
 * @param mac_addr MAC address of target sensor
 * @param command Command data to send
 * @param len Length of command data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_now_send_command(const uint8_t *mac_addr, const uint8_t *command, size_t len);

#ifdef __cplusplus
}
#endif