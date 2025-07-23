#ifndef DHT22_SENSOR_H
#define DHT22_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// DHT22 GPIO configuration
#define DHT22_GPIO_PIN 6

// Forward declaration for esp_err_t (will be properly included in .c file)
typedef int esp_err_t;

// DHT22 data structure
typedef struct {
    float temperature;  // Temperature in Celsius
    float humidity;     // Relative humidity in %
    bool valid;         // Data validity flag
} dht22_data_t;

/**
 * @brief Test DHT22 GPIO connection
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dht22_test_gpio(void);

/**
 * @brief Initialize DHT22 sensor
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dht22_init(void);

/**
 * @brief Read temperature and humidity from DHT22
 * 
 * @param data Pointer to store sensor data
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t dht22_read(dht22_data_t *data);

/**
 * @brief Get last valid sensor reading
 * 
 * @param data Pointer to store last valid data
 * @return esp_err_t ESP_OK if valid data available
 */
esp_err_t dht22_get_last_reading(dht22_data_t *data);

#endif // DHT22_SENSOR_H