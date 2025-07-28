#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include <stdbool.h>
#include "esp_err.h"

// Structure for outside sensor data
typedef struct {
    float temperature;
    float humidity;
    float temp_index;
    bool valid;
    char sensor_type[16];
} outside_sensor_data_t;

// Function declarations
esp_err_t http_client_init(void);
esp_err_t http_client_fetch_sensor_data(const char* url, outside_sensor_data_t* data);
void http_client_deinit(void);

#endif // HTTP_CLIENT_H