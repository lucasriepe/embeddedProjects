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
#include "dht22_sensor.h"  // DHT22 sensor integration
#include "http_client.h"  // HTTP client for remote sensor data

static const char *TAG = "weather_ui";

// URL for outside sensor data - now configurable
#define OUTSIDE_SENSOR_URL CONFIG_OUTSIDE_SENSOR_URL

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
static lv_obj_t *outside_status_label;  // Status indicator for remote sensor
static lv_obj_t *inside_box;
static lv_obj_t *inside_temp_label;
static lv_obj_t *inside_humidity_label;

// DHT22 sensor data
static dht22_data_t current_sensor_data = {0};
static outside_sensor_data_t outside_sensor_data = {0};

// Function declarations
static void wifi_init(void);
static void time_sync_init(void);
static void create_time_display(void);
static void create_weather_boxes(void);
static void update_time_display(lv_timer_t *timer);
static void update_sensor_display(lv_timer_t *timer);
static void update_remote_sensor_status(void);

// WiFi retry counter
static int wifi_retry_count = 0;

// WiFi Event Handler with configurable retry attempts
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi started, attempting connection");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < CONFIG_WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            wifi_retry_count++;
            wifi_connected = false;
            ESP_LOGI(TAG, "Retry to connect to the AP (attempt %d/%d)", 
                     wifi_retry_count, CONFIG_WIFI_MAXIMUM_RETRY);
        } else {
            ESP_LOGW(TAG, "Maximum retry attempts (%d) reached, stopping connection attempts", 
                     CONFIG_WIFI_MAXIMUM_RETRY);
            wifi_connected = false;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        wifi_retry_count = 0; // Reset retry counter on successful connection
        
        // Synchronize time when WiFi is connected
        time_sync_init();
    }
}

// WiFi initialization with configurable parameters
static void wifi_init(void)
{
    ESP_LOGI(TAG, "Initializing WiFi with configurable parameters");
    
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    
    // Set hostname from configuration
    ESP_ERROR_CHECK(esp_netif_set_hostname(sta_netif, CONFIG_WIFI_HOSTNAME));
    ESP_LOGI(TAG, "Device hostname set to: %s", CONFIG_WIFI_HOSTNAME);

    // WiFi configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
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

    // Configure WiFi connection parameters
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = CONFIG_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .scan_method = CONFIG_WIFI_SCAN_METHOD_FAST ? WIFI_FAST_SCAN : WIFI_ALL_CHANNEL_SCAN,
            .sort_method = CONFIG_WIFI_CONNECT_AP_BY_SIGNAL ? WIFI_CONNECT_AP_BY_SIGNAL : WIFI_CONNECT_AP_BY_SECURITY,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Configure power save mode if enabled
#ifdef CONFIG_WIFI_POWER_SAVE_MODE
    wifi_ps_type_t ps_type = WIFI_PS_MIN_MODEM;
#ifdef CONFIG_WIFI_POWER_SAVE_MAX_MODEM
    ps_type = WIFI_PS_MAX_MODEM;
#endif
    ESP_ERROR_CHECK(esp_wifi_set_ps(ps_type));
    ESP_LOGI(TAG, "WiFi power save mode enabled: %s", 
             ps_type == WIFI_PS_MIN_MODEM ? "Min Modem" : "Max Modem");
    
    // Set listen interval if power save is enabled
    wifi_config_t current_config;
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &current_config));
    current_config.sta.listen_interval = CONFIG_WIFI_LISTEN_INTERVAL;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &current_config));
    ESP_LOGI(TAG, "WiFi listen interval set to: %d", CONFIG_WIFI_LISTEN_INTERVAL);
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "WiFi power save mode disabled");
#endif
    
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization complete");
    ESP_LOGI(TAG, "Connecting to SSID: %s", CONFIG_WIFI_SSID);
    ESP_LOGI(TAG, "Maximum retry attempts: %d", CONFIG_WIFI_MAXIMUM_RETRY);
}

// Initialize time synchronization with configurable timezone and NTP servers
static void time_sync_init(void)
{
    ESP_LOGI(TAG, "Initializing SNTP with configurable timezone and NTP servers");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    
    // Use configured NTP servers from Kconfig
    esp_sntp_setservername(0, CONFIG_NTP_SERVER);
    esp_sntp_setservername(1, CONFIG_NTP_SERVER_BACKUP);
    esp_sntp_init();

    // Set timezone from Kconfig configuration
    setenv("TZ", CONFIG_TIMEZONE_POSIX, 1);
    tzset();

    // Wait for time synchronization (simplified)
    ESP_LOGI(TAG, "SNTP started, waiting for synchronization...");
    ESP_LOGI(TAG, "Using timezone: %s", CONFIG_TIMEZONE_POSIX);
    ESP_LOGI(TAG, "Primary NTP server: %s", CONFIG_NTP_SERVER);
    ESP_LOGI(TAG, "Backup NTP server: %s", CONFIG_NTP_SERVER_BACKUP);
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
    lv_obj_set_style_text_color(time_label, lv_color_black(), LV_PART_MAIN);
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

    // Outside status (remote sensor connection)
    outside_status_label = lv_label_create(outside_box);
    lv_label_set_text(outside_status_label, "Waiting...");
    lv_obj_set_style_text_font(outside_status_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(outside_status_label, lv_color_hex(0xf39c12), 0);  // Orange color
    lv_obj_align(outside_status_label, LV_ALIGN_BOTTOM_MID, 0, -10);

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

// Update sensor display with current DHT22 readings and outside sensor data
static void update_sensor_display(lv_timer_t *timer)
{
    // Update inside sensor (DHT22)
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
        
        ESP_LOGI(TAG, "Inside sensor updated: %.1f°C, %.1f%%", 
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
            ESP_LOGW(TAG, "Failed to read DHT22 sensor: error code %d", ret);
        }
    }
    
    // Update outside sensor data via HTTP
    update_remote_sensor_status();
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
        strftime(strftime_buf, sizeof(strftime_buf), "%H:%M", &timeinfo);
        lv_label_set_text(time_label, strftime_buf);
    } else {
        lv_label_set_text(time_label, "Time not set");
    }
}

// Update remote sensor status - now fetches data from HTTP server
static void update_remote_sensor_status(void)
{
    if (!outside_status_label) {
        return;
    }
    
    // Only try to fetch data if WiFi is connected
    if (!wifi_connected) {
        lv_label_set_text(outside_status_label, "No WiFi");
        lv_obj_set_style_text_color(outside_status_label, lv_color_hex(0xe74c3c), 0);  // Red color
        
        if (outside_temp_label) {
            lv_label_set_text(outside_temp_label, "--°C");
        }
        if (outside_humidity_label) {
            lv_label_set_text(outside_humidity_label, "--%");
        }
        return;
    }
    
    // Fetch sensor data from HTTP server
    esp_err_t ret = http_client_fetch_sensor_data(OUTSIDE_SENSOR_URL, &outside_sensor_data);
    
    if (ret == ESP_OK && outside_sensor_data.valid) {
        // Update status to show successful connection
        lv_label_set_text(outside_status_label, "Connected");
        lv_obj_set_style_text_color(outside_status_label, lv_color_hex(0x27ae60), 0);  // Green color
        
        // Update temperature and humidity labels with real data
        char temp_str[16];
        char humidity_str[16];
        
        snprintf(temp_str, sizeof(temp_str), "%.1f°C", outside_sensor_data.temperature);
        snprintf(humidity_str, sizeof(humidity_str), "%.1f%%", outside_sensor_data.humidity);
        
        if (outside_temp_label) {
            lv_label_set_text(outside_temp_label, temp_str);
        }
        if (outside_humidity_label) {
            lv_label_set_text(outside_humidity_label, humidity_str);
        }
        
        ESP_LOGI(TAG, "Outside sensor updated: %s, %.1f°C, %.1f%%", 
                 outside_sensor_data.sensor_type, outside_sensor_data.temperature, outside_sensor_data.humidity);
    } else {
        // Show error state
        lv_label_set_text(outside_status_label, "HTTP Error");
        lv_obj_set_style_text_color(outside_status_label, lv_color_hex(0xe74c3c), 0);  // Red color
        
        if (outside_temp_label) {
            lv_label_set_text(outside_temp_label, "--°C");
        }
        if (outside_humidity_label) {
            lv_label_set_text(outside_humidity_label, "--%");
        }
        
        ESP_LOGW(TAG, "Failed to fetch outside sensor data");
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
        ESP_LOGE(TAG, "DHT22 GPIO test failed: error code %d", ret);
    }
    
    // Initialize sensor
    ret = dht22_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DHT22 sensor: error code %d", ret);
    } else {
        ESP_LOGI(TAG, "DHT22 sensor initialized successfully");
    }
    
    // Create main screen
    main_screen = lv_obj_create(NULL);
    lv_scr_load(main_screen);
    
    // Set background color to white
    lv_obj_set_style_bg_color(main_screen, lv_color_white(), LV_PART_MAIN);
    
    // Create time display
    create_time_display();
    
    // Create weather boxes
    create_weather_boxes();
    
    // Initialize WiFi and start connection
    wifi_init();
    
    // Initialize HTTP client for outside sensor data
    ESP_LOGI(TAG, "Initializing HTTP client for outside sensor");
    esp_err_t http_ret = http_client_init();
    if (http_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client: %s", esp_err_to_name(http_ret));
    } else {
        ESP_LOGI(TAG, "HTTP client initialized successfully");
    }
    
    // Start time update timer (update every second)
    time_timer = lv_timer_create(update_time_display, 1000, NULL);
    
    // Start sensor update timer with configurable interval
    sensor_timer = lv_timer_create(update_sensor_display, CONFIG_SENSOR_UPDATE_INTERVAL_MS, NULL);
    ESP_LOGI(TAG, "Sensor update timer started with interval: %d ms", CONFIG_SENSOR_UPDATE_INTERVAL_MS);
    
    // Initial sensor reading
    update_sensor_display(NULL);
    
    ESP_LOGI(TAG, "Weather station UI initialized successfully");
}

// Cleanup function for weather station UI
void weather_station_ui_cleanup(void)
{
    // Stop timers
    if (time_timer) {
        lv_timer_del(time_timer);
        time_timer = NULL;
    }
    if (sensor_timer) {
        lv_timer_del(sensor_timer);
        sensor_timer = NULL;
    }
    
    // Cleanup HTTP client
    http_client_deinit();
    
    ESP_LOGI(TAG, "Weather station UI cleanup completed");
}