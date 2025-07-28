#include "http_client.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "HTTP_CLIENT";

// Buffer for HTTP response
#define MAX_HTTP_RECV_BUFFER 512
static char http_recv_buffer[MAX_HTTP_RECV_BUFFER];
static int http_recv_len = 0;

// HTTP event handler
esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Copy response data to buffer
                if (http_recv_len + evt->data_len < MAX_HTTP_RECV_BUFFER) {
                    memcpy(http_recv_buffer + http_recv_len, evt->data, evt->data_len);
                    http_recv_len += evt->data_len;
                    http_recv_buffer[http_recv_len] = '\0';
                }
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

esp_err_t http_client_init(void)
{
    ESP_LOGI(TAG, "HTTP Client initialized");
    return ESP_OK;
}

esp_err_t http_client_fetch_sensor_data(const char* url, outside_sensor_data_t* data)
{
    if (!data || !url) {
        return ESP_ERR_INVALID_ARG;
    }

    // Reset buffer
    memset(http_recv_buffer, 0, sizeof(http_recv_buffer));
    http_recv_len = 0;

    // Configure HTTP client
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return ESP_FAIL;
    }

    // Perform GET request
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        int content_length = esp_http_client_get_content_length(client);
        
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d", status_code, content_length);
        
        if (status_code == 200 && http_recv_len > 0) {
            ESP_LOGI(TAG, "Received data: %s", http_recv_buffer);
            
            // Replace single quotes with double quotes for proper JSON parsing
            char *json_str = malloc(strlen(http_recv_buffer) + 1);
            if (json_str == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for JSON string conversion");
                esp_http_client_cleanup(client);
                return ESP_FAIL;
            }
            
            // Convert single quotes to double quotes
            int i, j = 0;
            for (i = 0; http_recv_buffer[i] != '\0'; i++) {
                if (http_recv_buffer[i] == '\'') {
                    json_str[j++] = '"';
                } else {
                    json_str[j++] = http_recv_buffer[i];
                }
            }
            json_str[j] = '\0';
            
            ESP_LOGI(TAG, "Converted JSON: %s", json_str);
            
            // Parse JSON response
             cJSON *json = cJSON_Parse(json_str);
             if (json) {
                 cJSON *sensor = cJSON_GetObjectItem(json, "sensor");
                 cJSON *temp = cJSON_GetObjectItem(json, "temp");
                 cJSON *humidity = cJSON_GetObjectItem(json, "humI");  // Changed from "humidity" to "humI"
                 cJSON *temp_index = cJSON_GetObjectItem(json, "tmpIndex");
                
                if (sensor && temp && humidity && temp_index) {
                    // Copy sensor type
                    strncpy(data->sensor_type, cJSON_GetStringValue(sensor), sizeof(data->sensor_type) - 1);
                    data->sensor_type[sizeof(data->sensor_type) - 1] = '\0';
                    
                    // Copy sensor values
                    data->temperature = (float)cJSON_GetNumberValue(temp);
                    data->humidity = (float)cJSON_GetNumberValue(humidity);
                    data->temp_index = (float)cJSON_GetNumberValue(temp_index);
                    data->valid = true;
                    
                    ESP_LOGI(TAG, "Parsed sensor data: %s, Temp: %.1f°C, Humidity: %.1f%%, TempIndex: %.1f", 
                             data->sensor_type, data->temperature, data->humidity, data->temp_index);
                    
                    cJSON_Delete(json);
                    free(json_str);
                    esp_http_client_cleanup(client);
                    return ESP_OK;
                } else {
                    ESP_LOGE(TAG, "Missing required fields in JSON response");
                }
                cJSON_Delete(json);
            } else {
                ESP_LOGE(TAG, "Failed to parse JSON response");
            }
            free(json_str);
        } else {
            ESP_LOGE(TAG, "HTTP request failed with status %d", status_code);
        }
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    // Mark data as invalid on error
    data->valid = false;
    esp_http_client_cleanup(client);
    return ESP_FAIL;
}

void http_client_deinit(void)
{
    ESP_LOGI(TAG, "HTTP Client deinitialized");
}