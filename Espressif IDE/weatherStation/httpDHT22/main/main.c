#include <stdio.h>

//*WIFI header
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

//*HTTP header
#include <esp_http_server.h>

//*WiFi setup
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

void wifi_init(void)
{
//WIFI--------------------------------------------------------------------------------------
//*NVS init
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);

//*WIFI init
ESP_ERROR_CHECK(esp_event_loop_create_default());

wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
ESP_ERROR_CHECK(esp_wifi_init(&cfg));


//*WIFI config
wifi_config_t wifi_config = {
    .sta = {
        .ssid = EXAMPLE_WIFI_SSID,
        .password = EXAMPLE_WIFI_PASS,
    },
};
ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

//*WIFI connection
ESP_ERROR_CHECK(esp_wifi_start());
ESP_ERROR_CHECK(esp_wifi_connect());

//WIFI-END-------------------------------------------------------------------------------------
}

static esp_err_t hello_get_handler(httpd_req_t *req) {
    const char resp[] = "Hello, World!";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_uri_t hello = {
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    .user_ctx  = NULL
};

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &hello);
    }
}

void app_main(void)
{
    //?WiFi start
    wifi_init();

    //?HTTP server start
    //start_webserver();
}