#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "wifi_config.h"

// WiFi configuration is now loaded from wifi_config.h (generated from .env)
#define WIFI_MAXIMUM_RETRY  10
#define WIFI_CONNECT_TIMEOUT_MS 30000  // 30 seconds timeout

// Event Group for WiFi
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "HTTP_SENSOR";
static int s_retry_num = 0;

// WiFi Event Handler
static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started, attempting to connect...");
        
        // Perform WiFi scan to check if target SSID is available
        wifi_scan_and_check();
        
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        
        // Detailed disconnect reason logging
        switch(disconnected->reason) {
            case WIFI_REASON_AUTH_EXPIRE:
                ESP_LOGW(TAG, "WiFi disconnected: Authentication expired (reason: %d)", disconnected->reason);
                ESP_LOGW(TAG, "This usually means wrong password or security settings");
                break;
            case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
                ESP_LOGW(TAG, "WiFi disconnected: 4-way handshake timeout (reason: %d)", disconnected->reason);
                ESP_LOGW(TAG, "Check password and security settings");
                break;
            case WIFI_REASON_BEACON_TIMEOUT:
                ESP_LOGW(TAG, "WiFi disconnected: Beacon timeout (reason: %d)", disconnected->reason);
                ESP_LOGW(TAG, "Router may be too far away or signal too weak");
                break;
            case WIFI_REASON_NO_AP_FOUND:
                ESP_LOGW(TAG, "WiFi disconnected: No AP found (reason: %d)", disconnected->reason);
                ESP_LOGW(TAG, "Check if SSID '%s' is correct and broadcasting", WIFI_SSID);
                break;
            case WIFI_REASON_ASSOC_FAIL:
                ESP_LOGW(TAG, "WiFi disconnected: Association failed (reason: %d)", disconnected->reason);
                ESP_LOGW(TAG, "Router may be rejecting connection - check MAC filtering");
                break;
            default:
                ESP_LOGW(TAG, "WiFi disconnected, reason: %d", disconnected->reason);
                break;
        }
        
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d)", s_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            ESP_LOGE(TAG, "Failed to connect after %d attempts", WIFI_MAXIMUM_RETRY);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected! Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// WiFi scan function to check if target SSID is available
static void wifi_scan_and_check(void)
{
    ESP_LOGI(TAG, "Starting WiFi scan to check for available networks...");
    
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi scan failed: %s", esp_err_to_name(err));
        return;
    }
    
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    if (ap_count == 0) {
        ESP_LOGW(TAG, "No WiFi networks found!");
        return;
    }
    
    wifi_ap_record_t *ap_info = malloc(sizeof(wifi_ap_record_t) * ap_count);
    if (ap_info == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for AP scan results");
        return;
    }
    
    esp_wifi_scan_get_ap_records(&ap_count, ap_info);
    
    ESP_LOGI(TAG, "Found %d WiFi networks:", ap_count);
    bool target_found = false;
    
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGI(TAG, "  %d: SSID: %s, RSSI: %d, Auth: %d", 
                 i+1, ap_info[i].ssid, ap_info[i].rssi, ap_info[i].authmode);
        
        if (strcmp((char*)ap_info[i].ssid, WIFI_SSID) == 0) {
            target_found = true;
            ESP_LOGI(TAG, "  *** Target SSID '%s' found! RSSI: %d, Auth: %d ***", 
                     WIFI_SSID, ap_info[i].rssi, ap_info[i].authmode);
        }
    }
    
    if (!target_found) {
        ESP_LOGW(TAG, "Target SSID '%s' not found in scan results!", WIFI_SSID);
        ESP_LOGW(TAG, "Please check:");
        ESP_LOGW(TAG, "  1. SSID spelling is correct");
        ESP_LOGW(TAG, "  2. Router is powered on and broadcasting");
        ESP_LOGW(TAG, "  3. ESP32 is within range of the router");
    }
    
    free(ap_info);
}

// WiFi Initialization
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_LOGI(TAG, "Initializing WiFi...");
    ESP_LOGI(TAG, "Target SSID: %s", WIFI_SSID);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN,  // Accept any auth mode
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
            .bssid_set = false,
        },
    };
    
    ESP_LOGI(TAG, "Setting WiFi configuration...");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization completed. Waiting for connection...");

    // Wait for connection or failure with longer timeout
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to AP SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to AP SSID: %s", WIFI_SSID);
        ESP_LOGE(TAG, "Please check:");
        ESP_LOGE(TAG, "1. WiFi network is available and broadcasting");
        ESP_LOGE(TAG, "2. SSID and password are correct");
        ESP_LOGE(TAG, "3. ESP32 is within range of the router");
        ESP_LOGE(TAG, "4. Router supports 2.4GHz (ESP32 doesn't support 5GHz)");
    } else {
        ESP_LOGE(TAG, "Connection timeout after %d ms", WIFI_CONNECT_TIMEOUT_MS);
    }
}

// HTTP GET Handler for /sensor route
static esp_err_t sensor_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /sensor called");
    
    const char* resp_str = "Hello World";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

// HTTP Server Configuration
static const httpd_uri_t sensor = {
    .uri       = "/sensor",
    .method    = HTTP_GET,
    .handler   = sensor_get_handler,
    .user_ctx  = NULL
};

// Start HTTP Server
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &sensor);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 HTTP Sensor Server starting...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize and connect WiFi
    wifi_init_sta();

    // Check if WiFi connection was successful
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully, starting HTTP server...");
        
        // Start HTTP Server only if WiFi is connected
        static httpd_handle_t server = NULL;
        server = start_webserver();
        
        if (server) {
            ESP_LOGI(TAG, "HTTP Server started successfully!");
            ESP_LOGI(TAG, "Sensor data available at: http://[ESP32-IP]/sensor");
        } else {
            ESP_LOGE(TAG, "Failed to start HTTP Server!");
        }
    } else {
        ESP_LOGE(TAG, "WiFi connection failed! HTTP server will not start.");
        ESP_LOGE(TAG, "Please check your WiFi credentials in the .env file");
        ESP_LOGE(TAG, "Run 'python3 generate_wifi_config.py' after updating .env");
    }

    // Main loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds
    }
}