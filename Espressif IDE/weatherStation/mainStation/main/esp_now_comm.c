/*
 * ESP-NOW Communication Module Implementation
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_now_comm.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "ESP_NOW_COMM";

// Global variables
static esp_now_data_callback_t g_data_callback = NULL;
static esp_now_remote_sensor_t g_remote_sensors[ESP_NOW_MAX_REMOTE_SENSORS];
static uint8_t g_sensor_count = 0;
static SemaphoreHandle_t g_sensor_mutex = NULL;
static bool g_esp_now_initialized = false;

// Internal function declarations
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static esp_err_t esp_now_add_peer_if_not_exists(const uint8_t *mac_addr);

esp_err_t esp_now_comm_init(esp_now_data_callback_t data_callback)
{
    if (g_esp_now_initialized) {
        ESP_LOGW(TAG, "ESP-NOW already initialized");
        return ESP_OK;
    }

    if (data_callback == NULL) {
        ESP_LOGE(TAG, "Data callback cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Create mutex for sensor data protection
    g_sensor_mutex = xSemaphoreCreateMutex();
    if (g_sensor_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize ESP-NOW
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW: error code %d", ret);
        vSemaphoreDelete(g_sensor_mutex);
        return ret;
    }

    // Register callbacks
    ret = esp_now_register_recv_cb(esp_now_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback: error code %d", ret);
        esp_now_deinit();
        vSemaphoreDelete(g_sensor_mutex);
        return ret;
    }

    ret = esp_now_register_send_cb(esp_now_send_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register send callback: error code %d", ret);
        esp_now_deinit();
        vSemaphoreDelete(g_sensor_mutex);
        return ret;
    }

    // Store callback and initialize sensor array
    g_data_callback = data_callback;
    memset(g_remote_sensors, 0, sizeof(g_remote_sensors));
    g_sensor_count = 0;
    g_esp_now_initialized = true;

    ESP_LOGI(TAG, "ESP-NOW communication initialized successfully");
    return ESP_OK;
}

esp_err_t esp_now_comm_deinit(void)
{
    if (!g_esp_now_initialized) {
        return ESP_OK;
    }

    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();

    if (g_sensor_mutex) {
        vSemaphoreDelete(g_sensor_mutex);
        g_sensor_mutex = NULL;
    }

    g_data_callback = NULL;
    g_sensor_count = 0;
    g_esp_now_initialized = false;

    ESP_LOGI(TAG, "ESP-NOW communication deinitialized");
    return ESP_OK;
}

esp_err_t esp_now_register_sensor(const uint8_t *mac_addr, uint8_t sensor_id, const char *name)
{
    if (!g_esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mac_addr == NULL || name == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take sensor mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Check if sensor already exists
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        if (g_remote_sensors[i].sensor_id == sensor_id) {
            ESP_LOGW(TAG, "Sensor ID %d already registered", sensor_id);
            xSemaphoreGive(g_sensor_mutex);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Check if we have space for more sensors
    if (g_sensor_count >= ESP_NOW_MAX_REMOTE_SENSORS) {
        ESP_LOGE(TAG, "Maximum number of sensors reached");
        xSemaphoreGive(g_sensor_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Add new sensor
    esp_now_remote_sensor_t *sensor = &g_remote_sensors[g_sensor_count];
    memcpy(sensor->mac_addr, mac_addr, 6);
    sensor->sensor_id = sensor_id;
    strncpy(sensor->name, name, sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = '\0';
    sensor->is_active = false;
    sensor->last_seen = 0;
    memset(&sensor->last_data, 0, sizeof(sensor->last_data));

    g_sensor_count++;

    xSemaphoreGive(g_sensor_mutex);

    // Add as ESP-NOW peer
    esp_err_t ret = esp_now_add_peer_if_not_exists(mac_addr);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add peer, but sensor registered: error code %d", ret);
    }

    ESP_LOGI(TAG, "Registered sensor '%s' with ID %d", name, sensor_id);
    return ESP_OK;
}

esp_err_t esp_now_get_sensor_info(uint8_t sensor_id, esp_now_remote_sensor_t *sensor_info)
{
    if (!g_esp_now_initialized || sensor_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < g_sensor_count; i++) {
        if (g_remote_sensors[i].sensor_id == sensor_id) {
            memcpy(sensor_info, &g_remote_sensors[i], sizeof(esp_now_remote_sensor_t));
            xSemaphoreGive(g_sensor_mutex);
            return ESP_OK;
        }
    }

    xSemaphoreGive(g_sensor_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t esp_now_get_all_sensors(esp_now_remote_sensor_t *sensors, uint8_t max_sensors, uint8_t *count)
{
    if (!g_esp_now_initialized || sensors == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t copy_count = (g_sensor_count < max_sensors) ? g_sensor_count : max_sensors;
    memcpy(sensors, g_remote_sensors, copy_count * sizeof(esp_now_remote_sensor_t));
    *count = copy_count;

    xSemaphoreGive(g_sensor_mutex);
    return ESP_OK;
}

bool esp_now_is_sensor_online(uint8_t sensor_id, uint32_t timeout_ms)
{
    if (!g_esp_now_initialized) {
        return false;
    }

    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool is_online = false;

    for (uint8_t i = 0; i < g_sensor_count; i++) {
        if (g_remote_sensors[i].sensor_id == sensor_id) {
            is_online = g_remote_sensors[i].is_active && 
                       (current_time - g_remote_sensors[i].last_seen) < timeout_ms;
            break;
        }
    }

    xSemaphoreGive(g_sensor_mutex);
    return is_online;
}

esp_err_t esp_now_send_command(const uint8_t *mac_addr, const uint8_t *command, size_t len)
{
    if (!g_esp_now_initialized || mac_addr == NULL || command == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_now_add_peer_if_not_exists(mac_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer for command: error code %d", ret);
        return ret;
    }

    ret = esp_now_send(mac_addr, command, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: error code %d", ret);
    }

    return ret;
}

// Internal callback for received data
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len != sizeof(esp_now_sensor_data_t)) {
        ESP_LOGW(TAG, "Invalid received data: len=%d, expected=%d", len, sizeof(esp_now_sensor_data_t));
        return;
    }

    esp_now_sensor_data_t *sensor_data = (esp_now_sensor_data_t *)data;
    
    // Basic validation
    if (!sensor_data->valid) {
        ESP_LOGW(TAG, "Received invalid sensor data");
        return;
    }

    // Update sensor information
    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        for (uint8_t i = 0; i < g_sensor_count; i++) {
            if (g_remote_sensors[i].sensor_id == sensor_data->sensor_id) {
                memcpy(&g_remote_sensors[i].last_data, sensor_data, sizeof(esp_now_sensor_data_t));
                g_remote_sensors[i].last_seen = current_time;
                g_remote_sensors[i].is_active = true;
                break;
            }
        }
        
        xSemaphoreGive(g_sensor_mutex);
    }

    // Call user callback
    if (g_data_callback) {
        g_data_callback(sensor_data, recv_info->src_addr);
    }

    ESP_LOGI(TAG, "Received data from sensor %d: T=%.1f°C, H=%.1f%%", 
             sensor_data->sensor_id, sensor_data->temperature, sensor_data->humidity);
}

// Internal callback for sent data
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "Send success to " MACSTR, MAC2STR(mac_addr));
    } else {
        ESP_LOGW(TAG, "Send failed to " MACSTR, MAC2STR(mac_addr));
    }
}

// Internal function to add peer if not exists
static esp_err_t esp_now_add_peer_if_not_exists(const uint8_t *mac_addr)
{
    if (esp_now_is_peer_exist(mac_addr)) {
        return ESP_OK;
    }

    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, mac_addr, 6);
    peer_info.channel = 0;  // Use current channel
    peer_info.encrypt = false;

    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer " MACSTR ": error code %d", MAC2STR(mac_addr), ret);
    }

    return ret;
}