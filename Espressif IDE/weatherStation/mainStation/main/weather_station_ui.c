/*
 * Custom Weather Station UI Implementation
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include "weather_station_ui.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "time.h"
#include "sys/time.h"
#include "wifi_config.h"  // Generated from .env file
#include "dht22_sensor.h"  // DHT22 sensor integration

static const char *TAG = "weather_ui";

// Global variables
static lv_obj_t *main_screen;
static lv_obj_t *time_label;
static lv_timer_t *time_timer;
static lv_timer_t *sensor_timer;  // Timer for sensor readings
static bool wifi_connected = false;

// Weather box UI elements
static lv_obj_t *outside_box;
static lv_obj_t *outside_temp_label;
static lv_obj_t *outside_humidity_label;
static lv_obj_t *inside_box;
static lv_obj_t *inside_temp_label;
static lv_obj_t *inside_humidity_label;

// DHT22 sensor data
static dht22_data_t current_sensor_data = {0};

// Function declarations
static void wifi_init(void);
static void time_sync_init(void);
static void create_time_display(void);
static void create_weather_boxes(void);
static void update_time_display(lv_timer_t *timer);
static void update_sensor_display(lv_timer_t *timer);

// WiFi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        wifi_connected = false;
        ESP_LOGI(TAG, "Retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        
        // Synchronize time when WiFi is connected
        time_sync_init();
    }
}

// Initialize WiFi
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    
    // Set hostname
    ESP_ERROR_CHECK(esp_netif_set_hostname(sta_netif, "ESPmainStation"));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization completed. Hostname: ESPmainStation, connecting to %s...", WIFI_SSID);
}

// Initialize time synchronization
static void time_sync_init(void)
{
    ESP_LOGI(TAG, "Initializing SNTP with configurable timezone");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "de.pool.ntp.org");  // German NTP servers
    esp_sntp_init();

    // Set timezone from configuration
    setenv("TZ", TIMEZONE_STRING, 1);
    tzset();

    // Wait for time synchronization (simplified)
    ESP_LOGI(TAG, "SNTP started, waiting for synchronization...");
    ESP_LOGI(TAG, "Using timezone: %s", TIMEZONE_STRING);
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait 5 seconds
    
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    if (timeinfo.tm_year > (2016 - 1900)) {
        ESP_LOGI(TAG, "Time successfully synchronized");
    } else {
        ESP_LOGW(TAG, "Time synchronization taking longer than expected");
    }
}

// Create time display
static void create_time_display(void)
{
    time_label = lv_label_create(main_screen);
    lv_label_set_text(time_label, "Loading time...");
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(time_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(time_label, LV_ALIGN_TOP_MID, 0, 20);
}

// Create weather boxes
static void create_weather_boxes(void)
{
    // Outside Box
    outside_box = lv_obj_create(main_screen);
    lv_obj_set_size(outside_box, 250, 250);
    lv_obj_align(outside_box, LV_ALIGN_CENTER, -180, 0);
    lv_obj_set_style_radius(outside_box, 20, LV_PART_MAIN);
    lv_obj_set_style_bg_color(outside_box, lv_color_hex(0x2c3e50), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(outside_box, 10, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(outside_box, lv_color_black(), LV_PART_MAIN);

    // Outside title
    lv_obj_t *outside_title = lv_label_create(outside_box);
    lv_label_set_text(outside_title, "Outside");
    lv_obj_set_style_text_font(outside_title, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(outside_title, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(outside_title, LV_ALIGN_TOP_MID, 0, 20);

    // Outside temperature
    outside_temp_label = lv_label_create(outside_box);
    lv_label_set_text(outside_temp_label, "0°C");
    lv_obj_set_style_text_font(outside_temp_label, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(outside_temp_label, lv_color_white(), 0);
    lv_obj_align(outside_temp_label, LV_ALIGN_CENTER, 0, -10);

    // Outside humidity
    outside_humidity_label = lv_label_create(outside_box);
    lv_label_set_text(outside_humidity_label, "0%");
    lv_obj_set_style_text_font(outside_humidity_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(outside_humidity_label, lv_color_white(), 0);
    lv_obj_align(outside_humidity_label, LV_ALIGN_CENTER, 0, 15);

    // Inside Box
    inside_box = lv_obj_create(main_screen);
    lv_obj_set_size(inside_box, 250, 250);
    lv_obj_align(inside_box, LV_ALIGN_CENTER, 180, 0);
    lv_obj_set_style_radius(inside_box, 20, LV_PART_MAIN);
    lv_obj_set_style_bg_color(inside_box, lv_color_hex(0x2c3e50), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(inside_box, 10, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(inside_box, lv_color_black(), LV_PART_MAIN);

    // Inside title
    lv_obj_t *inside_title = lv_label_create(inside_box);
    lv_label_set_text(inside_title, "Inside");
    lv_obj_set_style_text_font(inside_title, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(inside_title, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(inside_title, LV_ALIGN_TOP_MID, 0, 20);

    // Inside temperature
    inside_temp_label = lv_label_create(inside_box);
    lv_label_set_text(inside_temp_label, "0°C");
    lv_obj_set_style_text_font(inside_temp_label, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(inside_temp_label, lv_color_white(), 0);
    lv_obj_align(inside_temp_label, LV_ALIGN_CENTER, 0, -10);

    // Inside humidity
    inside_humidity_label = lv_label_create(inside_box);
    lv_label_set_text(inside_humidity_label, "0%");
    lv_obj_set_style_text_font(inside_humidity_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(inside_humidity_label, lv_color_white(), 0);
    lv_obj_align(inside_humidity_label, LV_ALIGN_CENTER, 0, 15);
}

// Update sensor display
static void update_sensor_display(lv_timer_t *timer)
{
    dht22_data_t sensor_data;
    esp_err_t ret = dht22_read(&sensor_data);
    
    if (ret == ESP_OK && sensor_data.valid) {
        // Update current sensor data
        current_sensor_data = sensor_data;
        
        // Update inside temperature and humidity labels
        char temp_str[16];
        char humidity_str[16];
        
        snprintf(temp_str, sizeof(temp_str), "%.1f°C", sensor_data.temperature);
        snprintf(humidity_str, sizeof(humidity_str), "%.1f%%", sensor_data.humidity);
        
        if (inside_temp_label) {
            lv_label_set_text(inside_temp_label, temp_str);
        }
        if (inside_humidity_label) {
            lv_label_set_text(inside_humidity_label, humidity_str);
        }
        
        ESP_LOGI(TAG, "Sensor data updated: %.1f°C, %.1f%%", 
                 sensor_data.temperature, sensor_data.humidity);
    } else {
        // Try to use last valid reading
        if (dht22_get_last_reading(&sensor_data) == ESP_OK) {
            char temp_str[16];
            char humidity_str[16];
            
            snprintf(temp_str, sizeof(temp_str), "%.1f°C", sensor_data.temperature);
            snprintf(humidity_str, sizeof(humidity_str), "%.1f%%", sensor_data.humidity);
            
            if (inside_temp_label) {
                lv_label_set_text(inside_temp_label, temp_str);
            }
            if (inside_humidity_label) {
                lv_label_set_text(inside_humidity_label, humidity_str);
            }
        } else {
            // Show error state
            if (inside_temp_label) {
                lv_label_set_text(inside_temp_label, "--°C");
            }
            if (inside_humidity_label) {
                lv_label_set_text(inside_humidity_label, "--%");
            }
            ESP_LOGW(TAG, "Failed to read DHT22 sensor: %s", esp_err_to_name(ret));
        }
    }
}

// Update time display
static void update_time_display(lv_timer_t *timer)
{
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year > (2016 - 1900)) {
        strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);
        lv_label_set_text(time_label, strftime_buf);
    } else {
        lv_label_set_text(time_label, "Time not set");
    }
}

void weather_station_ui_init(void)
{
    ESP_LOGI(TAG, "Initializing Weather Station UI");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize DHT22 sensor
    ESP_LOGI(TAG, "Initializing DHT22 sensor on GPIO %d", DHT22_GPIO_PIN);
    
    // First test GPIO connection
    ret = dht22_test_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DHT22 GPIO test failed: %s", esp_err_to_name(ret));
    }
    
    // Initialize sensor
    ret = dht22_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DHT22 sensor: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DHT22 sensor initialized successfully");
    }
    
    // Create main screen
    main_screen = lv_obj_create(NULL);
    lv_scr_load(main_screen);
    
    // Set background color to dark blue
    lv_obj_set_style_bg_color(main_screen, lv_color_hex(0x1a1a2e), LV_PART_MAIN);
    
    // Create time display
    create_time_display();
    
    // Create weather boxes
    create_weather_boxes();
    
    // Initialize WiFi and start connection
    wifi_init();
    
    // Start time update timer (update every second)
    time_timer = lv_timer_create(update_time_display, 1000, NULL);
    
    // Start sensor update timer (update every 5 seconds)
    sensor_timer = lv_timer_create(update_sensor_display, 5000, NULL);
    
    ESP_LOGI(TAG, "Weather Station UI initialized successfully");
}