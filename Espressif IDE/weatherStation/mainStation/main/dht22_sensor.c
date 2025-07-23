#include "dht22_sensor.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

static const char *TAG = "DHT22";

// Last valid reading storage
static dht22_data_t last_reading = {0};
static bool has_valid_reading = false;

// DHT22 timing constants (in microseconds)
#define DHT22_START_SIGNAL_LOW_TIME     1000    // 1ms
#define DHT22_START_SIGNAL_HIGH_TIME    30      // 30us
#define DHT22_RESPONSE_TIMEOUT          100     // 100us
#define DHT22_DATA_BIT_TIMEOUT          100     // 100us
#define DHT22_POWER_UP_TIME             2000000 // 2 seconds power-up time

// Minimum time between readings (DHT22 needs 2 seconds between readings)
#define DHT22_MIN_INTERVAL_US           2000000 // 2 seconds

static uint64_t last_reading_time = 0;

/**
 * @brief Wait for GPIO pin to reach specified level with timeout
 */
static esp_err_t wait_for_level(gpio_num_t pin, int level, uint32_t timeout_us)
{
    uint32_t start_time = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if ((esp_timer_get_time() - start_time) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
        ets_delay_us(1);
    }
    return ESP_OK;
}

/**
 * @brief Read one bit from DHT22
 */
static esp_err_t read_bit(gpio_num_t pin, uint8_t *bit)
{
    // Wait for low signal (start of bit)
    if (wait_for_level(pin, 0, DHT22_DATA_BIT_TIMEOUT) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Wait for high signal
    if (wait_for_level(pin, 1, DHT22_DATA_BIT_TIMEOUT) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Measure high signal duration
    uint32_t start_time = esp_timer_get_time();
    if (wait_for_level(pin, 0, DHT22_DATA_BIT_TIMEOUT) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }
    uint32_t duration = esp_timer_get_time() - start_time;
    
    // Determine bit value based on duration
    // 0: ~26-28us, 1: ~70us
    *bit = (duration > 40) ? 1 : 0;
    
    return ESP_OK;
}

esp_err_t dht22_test_gpio(void)
{
    ESP_LOGI(TAG, "Testing DHT22 GPIO connection...");
    
    gpio_num_t pin = (gpio_num_t)DHT22_GPIO_PIN;
    
    // Test 1: Set as output and toggle
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    
    ESP_LOGI(TAG, "Setting GPIO%d LOW", DHT22_GPIO_PIN);
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Setting GPIO%d HIGH", DHT22_GPIO_PIN);
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Test 2: Set as input and read level
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    int level = gpio_get_level(pin);
    ESP_LOGI(TAG, "GPIO%d input level: %d (should be 1 with pull-up)", DHT22_GPIO_PIN, level);
    
    // Restore output mode
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 1);
    
    ESP_LOGI(TAG, "GPIO test completed");
    return ESP_OK;
}

esp_err_t dht22_init(void)
{
    ESP_LOGI(TAG, "Initializing DHT22 sensor on GPIO%d", DHT22_GPIO_PIN);
    
    // Configure GPIO as output initially
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,  // Start as regular output
        .pin_bit_mask = (1ULL << DHT22_GPIO_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Enable internal pull-up
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set pin high initially and wait for sensor to stabilize
    gpio_set_level((gpio_num_t)DHT22_GPIO_PIN, 1);
    
    ESP_LOGI(TAG, "DHT22 sensor configured, waiting for stabilization...");
    
    // Wait for sensor power-up time (DHT22 needs time to stabilize)
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2 seconds
    
    ESP_LOGI(TAG, "DHT22 sensor initialized successfully");
    return ESP_OK;
}

esp_err_t dht22_read(dht22_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check minimum interval between readings
    uint64_t current_time = esp_timer_get_time();
    if (last_reading_time != 0 && (current_time - last_reading_time) < DHT22_MIN_INTERVAL_US) {
        ESP_LOGW(TAG, "DHT22 reading too frequent, minimum 2 seconds required");
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t raw_data[5] = {0};  // 40 bits = 5 bytes
    gpio_num_t pin = (gpio_num_t)DHT22_GPIO_PIN;
    
    ESP_LOGI(TAG, "Starting DHT22 read sequence on GPIO%d", DHT22_GPIO_PIN);
    
    // Disable interrupts during critical timing
    portDISABLE_INTERRUPTS();
    
    // Send start signal - pull low for at least 1ms
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    ets_delay_us(1200);  // 1.2ms low signal (DHT22 spec: min 1ms)
    
    // Release line and wait 20-40us
    gpio_set_level(pin, 1);
    ets_delay_us(30);    // 30us high signal
    
    // Switch to input mode
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    
    // Wait for DHT22 to pull line low (response signal)
    if (wait_for_level(pin, 0, DHT22_RESPONSE_TIMEOUT) != ESP_OK) {
        portENABLE_INTERRUPTS();
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 1);
        ESP_LOGE(TAG, "No response from DHT22 - check wiring and power");
        return ESP_ERR_TIMEOUT;
    }
    
    // Wait for DHT22 to pull line high
    if (wait_for_level(pin, 1, DHT22_RESPONSE_TIMEOUT) != ESP_OK) {
        portENABLE_INTERRUPTS();
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 1);
        ESP_LOGE(TAG, "Invalid response from DHT22 - response phase 1 failed");
        return ESP_ERR_TIMEOUT;
    }
    
    // Wait for DHT22 to pull line low again (start of data)
    if (wait_for_level(pin, 0, DHT22_RESPONSE_TIMEOUT) != ESP_OK) {
        portENABLE_INTERRUPTS();
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 1);
        ESP_LOGE(TAG, "Invalid response from DHT22 - response phase 2 failed");
        return ESP_ERR_TIMEOUT;
    }
    
    // Read 40 bits of data
    for (int i = 0; i < 40; i++) {
        uint8_t bit;
        if (read_bit(pin, &bit) != ESP_OK) {
            portENABLE_INTERRUPTS();
            gpio_set_direction(pin, GPIO_MODE_OUTPUT);
            gpio_set_level(pin, 1);
            ESP_LOGE(TAG, "Failed to read bit %d from DHT22", i);
            return ESP_ERR_TIMEOUT;
        }
        
        raw_data[i / 8] |= (bit << (7 - (i % 8)));
    }
    
    portENABLE_INTERRUPTS();
    
    // Switch back to output mode and set high
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 1);
    
    // Update last reading time
    last_reading_time = current_time;
    
    // Verify checksum
    uint8_t checksum = raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3];
    if (checksum != raw_data[4]) {
        ESP_LOGE(TAG, "DHT22 checksum error: calculated=0x%02X, received=0x%02X", checksum, raw_data[4]);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Convert raw data to temperature and humidity
    uint16_t humidity_raw = (raw_data[0] << 8) | raw_data[1];
    uint16_t temperature_raw = (raw_data[2] << 8) | raw_data[3];
    
    data->humidity = humidity_raw / 10.0f;
    
    // Handle negative temperatures
    if (temperature_raw & 0x8000) {
        data->temperature = -((temperature_raw & 0x7FFF) / 10.0f);
    } else {
        data->temperature = temperature_raw / 10.0f;
    }
    
    data->valid = true;
    
    // Store as last valid reading
    last_reading = *data;
    has_valid_reading = true;
    
    ESP_LOGI(TAG, "DHT22 reading successful: Temperature=%.1f°C, Humidity=%.1f%%", 
             data->temperature, data->humidity);
    
    return ESP_OK;
}

esp_err_t dht22_get_last_reading(dht22_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!has_valid_reading) {
        return ESP_ERR_NOT_FOUND;
    }
    
    *data = last_reading;
    return ESP_OK;
}