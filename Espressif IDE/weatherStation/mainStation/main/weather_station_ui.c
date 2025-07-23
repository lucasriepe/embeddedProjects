/*
 * Custom Weather Station UI Implementation
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl.h"
#include "esp_log.h"

static const char *TAG = "weather_ui";

// Main screen object
static lv_obj_t *main_screen;

// Function declaration
void weather_station_ui_init(void);

void weather_station_ui_init(void)
{
    ESP_LOGI(TAG, "Initializing empty GUI canvas");
    
    // Get current screen
    main_screen = lv_scr_act();
    
    // Set light background color
    lv_obj_set_style_bg_color(main_screen, lv_color_hex(0xF5F5F5), 0);
    
    ESP_LOGI(TAG, "Empty GUI canvas ready for design");
}