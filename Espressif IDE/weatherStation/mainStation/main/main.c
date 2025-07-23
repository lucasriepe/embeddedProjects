/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "waveshare_rgb_lcd_port.h"
#include "weather_station_ui.h"
#include "esp_log.h"

static const char *MAIN_TAG = "main";

// ESP-NOW example functions (from esp_now_example.c)
void register_example_remote_sensors(void);
void print_device_mac_address(void);

void app_main()
{
    waveshare_esp32_s3_rgb_lcd_init(); // Initialize the Waveshare ESP32-S3 RGB LCD 
    // wavesahre_rgb_lcd_bl_on();  //Turn on the screen backlight 
    // wavesahre_rgb_lcd_bl_off(); //Turn off the screen backlight 
    
    ESP_LOGI(MAIN_TAG, "Display Weather Station UI");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        // Initialize custom weather station UI (includes ESP-NOW)
        weather_station_ui_init();
        
        // Print this device's MAC address for reference
        print_device_mac_address();
        
        // Register example remote sensors
        // IMPORTANT: Update MAC addresses in esp_now_example.c with your actual device MACs!
        register_example_remote_sensors();
        
        // You can also use the built-in LVGL demos:
        // lv_demo_stress();
        // lv_demo_benchmark();
        // lv_demo_music();
        // lv_demo_widgets();
        // example_lvgl_demo_ui();
        
        // Release the mutex
        lvgl_port_unlock();
    }
}
