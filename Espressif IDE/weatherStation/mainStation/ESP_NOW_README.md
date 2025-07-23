# ESP-NOW Integration für Weather Station

Diese Erweiterung ermöglicht es der MainStation, Sensordaten von externen ESP8266/ESP32 Geräten über ESP-NOW zu empfangen.

## 🚀 Features

- **Drahtlose Kommunikation**: Empfang von Sensordaten ohne WiFi-Infrastruktur
- **Multi-Sensor Support**: Bis zu 10 Remote-Sensoren gleichzeitig
- **Automatische Status-Anzeige**: Online/Offline Status für jeden Sensor
- **Kompatibilität**: Funktioniert mit ESP8266 und ESP32
- **Energieeffizient**: Ideal für batteriebetriebene Außensensoren

## 📋 Verwendung

### 1. Remote-Sensoren registrieren

```c
// In main.c nach weather_station_ui_init()
uint8_t sensor_mac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
weather_station_register_remote_sensor(sensor_mac, 1, "Außensensor");
```

### 2. MAC-Adresse eines ESP-Geräts ermitteln

```c
uint8_t mac[6];
esp_wifi_get_mac(WIFI_IF_STA, mac);
printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
```

### 3. Remote-Sensor Code (ESP8266/ESP32)

```c
#include "esp_now.h"
#include "esp_wifi.h"

// Datenstruktur (muss identisch zur MainStation sein)
typedef struct {
    uint8_t sensor_id;
    float temperature;
    float humidity;
    uint32_t timestamp;
    uint16_t battery_voltage;
    bool valid;
} esp_now_sensor_data_t;

// MainStation MAC-Adresse (ersetzen Sie diese!)
uint8_t main_station_mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

void send_sensor_data(float temp, float hum) {
    esp_now_sensor_data_t data = {
        .sensor_id = 1,              // Eindeutige Sensor-ID
        .temperature = temp,
        .humidity = hum,
        .timestamp = time(NULL),
        .battery_voltage = 3300,     // Optional: Batteriespannung in mV
        .valid = true
    };
    
    esp_now_send(main_station_mac, (uint8_t*)&data, sizeof(data));
}
```

## 🔧 Konfiguration

### MainStation Setup

1. **ESP-NOW wird automatisch initialisiert** beim Start der UI
2. **Sensoren registrieren** mit `weather_station_register_remote_sensor()`
3. **Status überwachen** mit `weather_station_get_active_remote_sensors()`

### Remote-Sensor Setup

1. **WiFi initialisieren** (für ESP-NOW erforderlich)
2. **ESP-NOW initialisieren**
3. **MainStation als Peer hinzufügen**
4. **Sensordaten senden** in regelmäßigen Abständen

## 📊 UI-Anzeige

- **Outside Box**: Zeigt Daten vom ersten registrierten Remote-Sensor
- **Status-Indikator**: 
  - 🟡 **Waiting...** - Warten auf erste Daten
  - 🟢 **Online** - Sensor sendet aktiv Daten
  - 🔴 **Offline** - Sensor seit >60s nicht erreichbar

## ⚡ Energieoptimierung für Remote-Sensoren

```c
// ESP8266/ESP32 Deep Sleep zwischen Messungen
void enter_deep_sleep(uint32_t sleep_time_us) {
    ESP_LOGI(TAG, "Entering deep sleep for %d seconds", sleep_time_us / 1000000);
    esp_deep_sleep(sleep_time_us);
}

// Beispiel: Alle 5 Minuten messen und senden
void sensor_task() {
    // Sensor auslesen
    float temp, hum;
    read_dht22(&temp, &hum);
    
    // Daten senden
    send_sensor_data(temp, hum);
    
    // 5 Minuten schlafen
    enter_deep_sleep(5 * 60 * 1000000);
}
```

## 🔍 Debugging

### Log-Ausgaben aktivieren

```c
// In sdkconfig oder menuconfig
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_ESP_NOW_ENABLE=y
```

### Häufige Probleme

1. **Keine Daten empfangen**:
   - MAC-Adressen überprüfen
   - WiFi auf beiden Geräten initialisiert?
   - Gleiche Datenstruktur verwendet?

2. **Sensor zeigt "Offline"**:
   - Timeout-Wert anpassen (Standard: 60s)
   - Sendeintervall des Remote-Sensors prüfen

3. **Reichweite zu gering**:
   - Antennen optimieren
   - Hindernisse reduzieren
   - Sendeleistung erhöhen

## 📁 Dateien

- `esp_now_comm.h/c` - ESP-NOW Kommunikationsmodul
- `esp_now_example.c` - Beispielcode für Sensor-Registrierung
- `weather_station_ui.c` - Erweiterte UI mit ESP-NOW Integration

## 🔗 Weiterführende Links

- [ESP-NOW Dokumentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [ESP8266 ESP-NOW Guide](https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html#esp-now)
- [Energieoptimierung ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/deep-sleep-stub.html)