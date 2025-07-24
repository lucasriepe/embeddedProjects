# ESP-NOW Integration for Weather Station

This extension enables the MainStation to receive sensor data from external ESP8266/ESP32 devices via ESP-NOW.

## 🚀 Features

- **Wireless Communication**: Receive sensor data without WiFi infrastructure
- **Multi-Sensor Support**: Up to 10 remote sensors simultaneously
- **Automatic Status Display**: Online/Offline status for each sensor
- **Compatibility**: Works with ESP8266 and ESP32
- **Energy Efficient**: Ideal for battery-powered outdoor sensors

## 📋 Usage

### 1. Register Remote Sensors

```c
// In main.c after weather_station_ui_init()
uint8_t sensor_mac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
weather_station_register_remote_sensor(sensor_mac, 1, "Outdoor Sensor");
```

### 2. Get MAC Address of an ESP Device

```c
uint8_t mac[6];
esp_wifi_get_mac(WIFI_IF_STA, mac);
printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
```

### 3. Remote Sensor Code (ESP8266/ESP32)

```c
#include "esp_now.h"
#include "esp_wifi.h"

// Data structure (must be identical to MainStation)
typedef struct {
    uint8_t sensor_id;
    float temperature;
    float humidity;
    uint32_t timestamp;
    uint16_t battery_voltage;   // Battery voltage in mV (0 for mains-powered)
    bool valid;
} esp_now_sensor_data_t;

// MainStation MAC address (replace this!)
uint8_t main_station_mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

void send_sensor_data(float temp, float hum) {
    esp_now_sensor_data_t data = {
        .sensor_id = 1,              // Unique sensor ID
        .temperature = temp,
        .humidity = hum,
        .timestamp = time(NULL),
        .battery_voltage = 0,       // 0 for mains-powered sensors
        .valid = true
    };

    esp_now_send(main_station_mac, (uint8_t*)&data, sizeof(data));
}
```

## 🔧 Configuration

### MainStation Setup

1. **ESP-NOW is automatically initialized** when starting the UI
2. **Register sensors** with `weather_station_register_remote_sensor()`
3. **Monitor status** with `weather_station_get_active_remote_sensors()`

### Remote Sensor Setup

1. **Initialize WiFi** (required for ESP-NOW)
2. **Initialize ESP-NOW**
3. **Add MainStation as peer**
4. **Send sensor data** at regular intervals

## 📊 UI Display

- **Outside Box**: Shows data from the first registered remote sensor
- **Status Indicator**:
  - 🟡 **Waiting...** - Waiting for first data
  - 🟢 **Online** - Sensor actively sending data
  - 🔴 **Offline** - Sensor not reachable for >60s

## ⚡ Energy Optimization for Remote Sensors

```c
// ESP8266/ESP32 Deep Sleep between measurements
void enter_deep_sleep(uint32_t sleep_time_us) {
    ESP_LOGI(TAG, "Entering deep sleep for %d seconds", sleep_time_us / 1000000);
    esp_deep_sleep(sleep_time_us);
}

// Example: Measure and send every 5 minutes
void sensor_task() {
    // Read sensor
    float temp, hum;
    read_dht22(&temp, &hum);

    // Send data
    send_sensor_data(temp, hum);

    // Sleep for 5 minutes
    enter_deep_sleep(5 * 60 * 1000000);
}
```

## 🔍 Debugging

### Enable Log Output

```c
// In sdkconfig or menuconfig
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_ESP_NOW_ENABLE=y
```

### Common Issues

1. **No data received**:

   - Check MAC addresses
   - WiFi initialized on both devices?
   - Same data structure used?

2. **Sensor shows "Offline"**:

   - Adjust timeout value (default: 60s)
   - Check remote sensor send interval

3. **Range too short**:
   - Optimize antennas
   - Reduce obstacles
   - Increase transmission power

## 📁 Files

- `esp_now_comm.h/c` - ESP-NOW communication module
- `esp_now_example.c` - Example code for sensor registration
- `weather_station_ui.c` - Extended UI with ESP-NOW integration

## 🔗 Further Links

- [ESP-NOW Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [ESP8266 ESP-NOW Guide](https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html#esp-now)
- [ESP32 Energy Optimization](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/deep-sleep-stub.html)
