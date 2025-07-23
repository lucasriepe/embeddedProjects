/*
 * ESP-NOW Remote Sensor Registration Example
 * 
 * This file shows how to register remote ESP8266/ESP32 sensors
 * that will send data via ESP-NOW to the main station.
 * 
 * Add this code to your main.c or create a separate initialization function.
 */

#include "weather_station_ui.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_err.h"

static const char *TAG = "esp_now_example";

/**
 * @brief Register example remote sensors
 * 
 * Call this function after weather_station_ui_init() to register
 * remote sensors that will send data via ESP-NOW.
 */
void register_example_remote_sensors(void)
{
    ESP_LOGI(TAG, "Registering example remote sensors");
    
    // Example 1: ESP8266 outdoor sensor
    // Replace with actual MAC address of your ESP8266
    uint8_t esp8266_mac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    esp_err_t ret = weather_station_register_remote_sensor(esp8266_mac, 1, "Outdoor ESP8266");
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered ESP8266 outdoor sensor");
    } else {
        ESP_LOGE(TAG, "Failed to register ESP8266 sensor: error code %d", ret);
    }
    
    // Example 2: ESP32 garden sensor
    // Replace with actual MAC address of your ESP32
    uint8_t esp32_mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    ret = weather_station_register_remote_sensor(esp32_mac, 2, "Garden ESP32");
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered ESP32 garden sensor");
    } else {
        ESP_LOGE(TAG, "Failed to register ESP32 sensor: error code %d", ret);
    }
    
    ESP_LOGI(TAG, "Remote sensor registration completed");
}

/**
 * @brief Get MAC address of a device
 * 
 * Use this function to find out the MAC address of your ESP devices.
 * Run this on each ESP8266/ESP32 that you want to use as a remote sensor.
 */
void print_device_mac_address(void)
{
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Device MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/*
 * USAGE INSTRUCTIONS:
 * 
 * 1. In your main.c, after calling weather_station_ui_init(), add:
 *    register_example_remote_sensors();
 * 
 * 2. To find MAC addresses of your ESP devices, add this to their main():
 *    print_device_mac_address();
 * 
 * 3. Update the MAC addresses in register_example_remote_sensors() 
 *    with the actual MAC addresses of your remote sensors.
 * 
 * 4. On your remote ESP8266/ESP32 sensors, implement ESP-NOW sending
 *    using the esp_now_sensor_data_t structure defined in esp_now_comm.h
 * 
 * Example remote sensor code structure:
 * 
 * esp_now_sensor_data_t sensor_data = {
 *     .sensor_id = 1,              // Must match registered sensor ID
 *     .temperature = 23.5,         // Temperature in Celsius
 *     .humidity = 65.2,            // Humidity in percentage
 *     .timestamp = time(NULL),     // Current timestamp
 *     .battery_voltage = 3300,     // Battery voltage in mV (optional)
 *     .valid = true                // Data validity flag
 * };
 * 
 * esp_now_send(main_station_mac, (uint8_t*)&sensor_data, sizeof(sensor_data));
 */