/*
 * Custom Weather Station UI Implementation
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/apps/sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <time.h>
#include <sys/time.h>
#include "wifi_config.h"  // Generated from .env file

static const char *TAG = "weather_ui";

// Main screen object
static lv_obj_t *main_screen;
static lv_obj_t *time_label;
static lv_timer_t *time_timer;

// Weather display objects
static lv_obj_t *outside_box;
static lv_obj_t *inside_box;
static lv_obj_t *outside_temp_label;
static lv_obj_t *outside_humidity_label;
static lv_obj_t *inside_temp_label;
static lv_obj_t *inside_humidity_label;

// WiFi Status
static bool wifi_connected = false;

// Function declarations
void weather_station_ui_init(void);
static void wifi_init(void);
static void time_sync_init(void);
static void update_time_display(lv_timer_t *timer);
static void create_weather_boxes(void);

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
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(1, "de.pool.ntp.org");  // German NTP servers
    sntp_init();

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

// Update time display
static void update_time_display(lv_timer_t *timer)
{
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    time(&now);
    localtime_r(&now, &timeinfo);

    if (wifi_connected && timeinfo.tm_year > (2016 - 1900)) {
        // Show German time without seconds (only HH:MM)
        strftime(strftime_buf, sizeof(strftime_buf), "%H:%M", &timeinfo);
    } else {
        // Show status when not connected
        strcpy(strftime_buf, "--:--");
    }

    if (time_label) {
        lv_label_set_text(time_label, strftime_buf);
    }
}

// Create weather display boxes
static void create_weather_boxes(void)
{
    // Create Outside box
    outside_box = lv_obj_create(main_screen);
    lv_obj_set_size(outside_box, 250, 250);
    lv_obj_align(outside_box, LV_ALIGN_CENTER, -180, 20);
    lv_obj_set_style_bg_color(outside_box, lv_color_white(), 0);
    lv_obj_set_style_radius(outside_box, 15, 0);
    lv_obj_set_style_border_width(outside_box, 2, 0);
    lv_obj_set_style_border_color(outside_box, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_shadow_width(outside_box, 10, 0);
    lv_obj_set_style_shadow_color(outside_box, lv_color_hex(0x000000), 0);
    lv_obj_set_style_shadow_opa(outside_box, LV_OPA_20, 0);

    // Outside title
    lv_obj_t *outside_title = lv_label_create(outside_box);
    lv_label_set_text(outside_title, "Outside");
    lv_obj_set_style_text_font(outside_title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(outside_title, lv_color_hex(0x333333), 0);
    lv_obj_align(outside_title, LV_ALIGN_TOP_MID, 0, 10);

    // Outside temperature
    outside_temp_label = lv_label_create(outside_box);
    lv_label_set_text(outside_temp_label, "0°C");
    lv_obj_set_style_text_font(outside_temp_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(outside_temp_label, lv_color_hex(0x2196F3), 0);
    lv_obj_align(outside_temp_label, LV_ALIGN_CENTER, 0, -10);

    // Outside humidity
    outside_humidity_label = lv_label_create(outside_box);
    lv_label_set_text(outside_humidity_label, "0%");
    lv_obj_set_style_text_font(outside_humidity_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(outside_humidity_label, lv_color_hex(0x666666), 0);
    lv_obj_align(outside_humidity_label, LV_ALIGN_BOTTOM_MID, 0, -10);

    // Create Inside box
    inside_box = lv_obj_create(main_screen);
    lv_obj_set_size(inside_box, 250, 250);
    lv_obj_align(inside_box, LV_ALIGN_CENTER, 180, 20);
    lv_obj_set_style_bg_color(inside_box, lv_color_white(), 0);
    lv_obj_set_style_radius(inside_box, 15, 0);
    lv_obj_set_style_border_width(inside_box, 2, 0);
    lv_obj_set_style_border_color(inside_box, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_shadow_width(inside_box, 10, 0);
    lv_obj_set_style_shadow_color(inside_box, lv_color_hex(0x000000), 0);
    lv_obj_set_style_shadow_opa(inside_box, LV_OPA_20, 0);

    // Inside title
    lv_obj_t *inside_title = lv_label_create(inside_box);
    lv_label_set_text(inside_title, "Inside");
    lv_obj_set_style_text_font(inside_title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(inside_title, lv_color_hex(0x333333), 0);
    lv_obj_align(inside_title, LV_ALIGN_TOP_MID, 0, 10);

    // Inside temperature
    inside_temp_label = lv_label_create(inside_box);
    lv_label_set_text(inside_temp_label, "0°C");
    lv_obj_set_style_text_font(inside_temp_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(inside_temp_label, lv_color_hex(0xFF9800), 0);
    lv_obj_align(inside_temp_label, LV_ALIGN_CENTER, 0, -10);

    // Inside humidity
    inside_humidity_label = lv_label_create(inside_box);
    lv_label_set_text(inside_humidity_label, "0%");
    lv_obj_set_style_text_font(inside_humidity_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(inside_humidity_label, lv_color_hex(0x666666), 0);
    lv_obj_align(inside_humidity_label, LV_ALIGN_BOTTOM_MID, 0, -10);
}

void weather_station_ui_init(void)
{
    ESP_LOGI(TAG, "Initializing GUI with green header bar and time display");
    
    // Initialize NVS (for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize WiFi
    wifi_init();
    
    // Create main screen with light background
    main_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(main_screen, lv_color_hex(0xF5F5F5), 0);
    
    // Create green header bar in the upper center
    lv_obj_t *header_bar = lv_obj_create(main_screen);
    lv_obj_set_size(header_bar, 300, 50);  // Width: 300px, Height: 50px
    lv_obj_align(header_bar, LV_ALIGN_TOP_MID, 0, 20);  // Top center, 20px from top
    lv_obj_set_style_bg_color(header_bar, lv_color_hex(0x4CAF50), 0);  // Leaf green color
    lv_obj_set_style_radius(header_bar, 10, 0);  // Rounded corners
    lv_obj_set_style_border_width(header_bar, 0, 0);  // No border
    
    // Create "Weatherstation" label in the green bar
    lv_obj_t *title_label = lv_label_create(header_bar);
    lv_label_set_text(title_label, "Weatherstation");
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title_label, lv_color_white(), 0);
    lv_obj_center(title_label);  // Center the text in the bar
    
    // Create time display in top right corner
    time_label = lv_label_create(main_screen);
    lv_label_set_text(time_label, "--:--");
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(time_label, lv_color_black(), 0);
    lv_obj_align(time_label, LV_ALIGN_TOP_RIGHT, -20, 20);  // Top right, 20px from edges
    
    // Create weather boxes
    create_weather_boxes();
    
    // Start timer to update time every second
    time_timer = lv_timer_create(update_time_display, 1000, NULL);
    
    // Load the screen
    lv_scr_load(main_screen);
    
    ESP_LOGI(TAG, "GUI with green header bar and time display ready");
}