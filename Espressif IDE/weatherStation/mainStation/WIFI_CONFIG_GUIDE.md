# WiFi Configuration Guide

This guide explains how to configure WiFi settings for the Weather Station using the ESP-IDF configuration system.

## Configuration Menu

Use `idf.py menuconfig` to access the configuration menu. Navigate to **Weather Station Configuration** → **WiFi Configuration**.

## Available WiFi Settings

### Basic Connection Settings
- **WiFi SSID**: Your WiFi network name
- **WiFi Password**: Your WiFi network password
- **Device Hostname**: Custom hostname for the device (default: "weather-station")

### Connection Behavior
- **Maximum Retry Attempts**: Number of connection attempts before giving up (1-20, default: 5)
- **WiFi Scan Method**: 
  - Fast Scan: Quick connection to first matching AP
  - All Channel Scan: Comprehensive scan of all channels

### Advanced Settings
- **AP Sort Method**:
  - By Signal Strength: Connect to strongest signal
  - By Security: Prioritize security level

- **Power Save Mode**:
  - Disabled: Maximum performance, higher power consumption
  - Min Modem: Balanced power saving
  - Max Modem: Maximum power saving

- **Listen Interval**: Power save listen interval (1-100, default: 3)

### HTTP Client Settings
- **HTTP Client Timeout**: Timeout for HTTP requests in milliseconds (1000-30000ms, default: 5000ms)
- **Outside Sensor URL**: URL for the external sensor HTTP endpoint
- **Sensor Update Interval**: How often to update sensor data in milliseconds (1000-60000ms, default: 10000ms)

## Code Integration

The configuration values are automatically used in the code:

### WiFi Initialization
```c
// Hostname configuration
esp_netif_set_hostname(sta_netif, CONFIG_WIFI_HOSTNAME);

// Connection parameters
wifi_config_t wifi_config = {
    .sta = {
        .ssid = CONFIG_WIFI_SSID,
        .password = CONFIG_WIFI_PASSWORD,
        .scan_method = CONFIG_WIFI_SCAN_METHOD_FAST ? WIFI_FAST_SCAN : WIFI_ALL_CHANNEL_SCAN,
        .sort_method = CONFIG_WIFI_CONNECT_AP_BY_SIGNAL ? WIFI_CONNECT_AP_BY_SIGNAL : WIFI_CONNECT_AP_BY_SECURITY,
    },
};
```

### Retry Logic
```c
if (wifi_retry_count < CONFIG_WIFI_MAXIMUM_RETRY) {
    esp_wifi_connect();
    wifi_retry_count++;
    // ... retry logic
}
```

### HTTP Client Configuration
```c
esp_http_client_config_t config = {
    .url = CONFIG_OUTSIDE_SENSOR_URL,
    .timeout_ms = CONFIG_HTTP_CLIENT_TIMEOUT_MS,
};
```

### Timer Configuration
```c
sensor_timer = lv_timer_create(update_sensor_display, CONFIG_SENSOR_UPDATE_INTERVAL_MS, NULL);
```

## Configuration Examples

### Home Network (Balanced)
- SSID: "MyHomeWiFi"
- Password: "MyPassword123"
- Hostname: "weather-station"
- Max Retry: 5
- Scan Method: Fast Scan
- Power Save: Min Modem
- HTTP Timeout: 5000ms
- Update Interval: 10000ms (10 seconds)

### Office Network (Performance)
- SSID: "OfficeWiFi"
- Password: "OfficePassword"
- Hostname: "office-weather"
- Max Retry: 10
- Scan Method: All Channel Scan
- Power Save: Disabled
- HTTP Timeout: 3000ms
- Update Interval: 5000ms (5 seconds)

### Battery Powered (Power Saving)
- SSID: "BatteryWiFi"
- Password: "BatteryPassword"
- Hostname: "battery-weather"
- Max Retry: 3
- Scan Method: Fast Scan
- Power Save: Max Modem
- HTTP Timeout: 10000ms
- Update Interval: 30000ms (30 seconds)

## Benefits

1. **User-Friendly**: Configure via menuconfig instead of editing code
2. **Flexible**: Adapt to different network environments
3. **Power Efficient**: Configurable power saving options
4. **Reliable**: Configurable retry and timeout settings
5. **Documented**: Clear descriptions for all options
6. **Compile-Time**: Values are set at build time for efficiency

## Troubleshooting

### Connection Issues
- Increase **Maximum Retry Attempts**
- Try **All Channel Scan** method
- Check SSID and password spelling

### Power Consumption
- Enable **Power Save Mode**
- Increase **Sensor Update Interval**
- Increase **Listen Interval**

### HTTP Timeouts
- Increase **HTTP Client Timeout**
- Check network stability
- Verify **Outside Sensor URL**

### Performance Issues
- Disable **Power Save Mode**
- Decrease **Sensor Update Interval**
- Use **Fast Scan** method