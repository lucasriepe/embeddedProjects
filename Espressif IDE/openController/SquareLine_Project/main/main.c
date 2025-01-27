#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

//Driver
#include "driver/i2c.h"
#include "esp_lcd_touch_gt911.h"

//including the UI
#include "ui/ui.h"
//importing the file
#include "../components/espressif__esp_lcd_touch/display.h"

//--------------------------------------------------------
//main

void app_main(void){
	display();
    
}