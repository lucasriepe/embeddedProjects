# HTTP Client Implementation for Outside Sensor

## Overview

This implementation replaces ESP-NOW communication with HTTP-based data retrieval for the outside sensor. The HTTP client fetches sensor data from an ESP32-S3 server available at `http://192.168.178.59/dht`.

## Implemented Files

### 1. `http_client.h`
- Defines the data structure `outside_sensor_data_t` for outside sensor data
- Declares HTTP client functions for initialization, data retrieval, and cleanup

### 2. `http_client.c`
- Implements HTTP client using ESP-IDF HTTP Client API
- Parses JSON response in the format: `{"sensor": "DHT", "humidity": 65.20, "temperature": 23.50, "heatIndex": 24.15, "status": "success"}`
- Handles HTTP errors and network issues
- Validates response status field for successful data retrieval

### 3. Updated `weather_station_ui.c`
- Integrates HTTP client for outside sensor data
- Updates UI every 10 seconds with new data
- Displays connection status (Connected/No WiFi/HTTP Error)
- Handles error scenarios gracefully

## How It Works

1. **Initialization**: HTTP client is initialized when the Weather Station starts
2. **Data Retrieval**: Every 10 seconds, `http_client_fetch_sensor_data()` is called
3. **JSON Parsing**: Response is parsed and stored in the `outside_sensor_data_t` structure
4. **UI Update**: Temperature, humidity, and status are updated on the display

## Configuration

- **Server URL**: `http://192.168.178.59/dht` (defined in `OUTSIDE_SENSOR_URL`)
- **Update Interval**: 10 seconds
- **Timeout**: 5 seconds for HTTP requests

## Error Handling

- **No WiFi**: Displays "No WiFi" status
- **HTTP Error**: Displays "HTTP Error" status
- **JSON Parse Error**: Marks data as invalid
- **Timeout**: Automatic retry during the next update cycle

## Dependencies

- `esp_http_client`: ESP-IDF HTTP Client component
- `json`: ESP-IDF JSON Parser component
- Both have been added to `CMakeLists.txt`

## Status Display

- 🟢 **Connected**: Successful data retrieval
- 🔴 **No WiFi**: No WiFi connection
- 🔴 **HTTP Error**: HTTP request failed

The implementation is robust and handles all typical network and communication errors gracefully.