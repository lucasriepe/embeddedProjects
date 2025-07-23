idf_component_register(
    SRCS "esp_now_example.c" "esp_now_comm.c" "waveshare_rgb_lcd_port.c" "main.c" "lvgl_port.c" "weather_station_ui.c" "dht22_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES lvgl esp_wifi nvs_flash esp_netif esp_event driver)

idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
target_compile_options(${lvgl_lib} PRIVATE -Wno-format)
