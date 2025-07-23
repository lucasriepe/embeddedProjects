# ESP8266 Remote Weather Sensor

This Arduino IDE project turns an ESP8266 into a wireless weather sensor that sends data via ESP-NOW to the MainStation.

## 🔧 Hardware Setup

### Required Components
- ESP8266 (NodeMCU, Wemos D1 Mini, etc.)
- DHT22 Temperature/Humidity Sensor
- Breadboard and jumper wires
- Optional: Battery for mobile operation

### DHT22 Wiring
```
DHT22    →  ESP8266
VCC      →  VIN (5V)     ⚠️ IMPORTANT: Use 5V!
GND      →  GND
DATA     →  GPIO2 (D4 on NodeMCU)
```

**Important Note:** 
- The DHT22 requires 5V for reliable operation
- Use the VIN pin on ESP8266 (not 3.3V!)
- GPIO2 is 5V-tolerant, so data line works fine
- With USB power supply, 5V is available at VIN pin

## 📚 Software Setup

### 1. Install Arduino IDE Libraries
```
- ESP8266WiFi (pre-installed)
- DHT sensor library by Adafruit
```

**Installation:**
1. Arduino IDE → Tools → Manage Libraries
2. Search for "DHT sensor library"
3. Install "DHT sensor library by Adafruit"

### 2. Board Configuration
```
Board: NodeMCU 1.0 (ESP-12E Module)
Upload Speed: 115200
CPU Frequency: 80 MHz
Flash Size: 4MB (FS:2MB OTA:~1019KB)
```

## 🚀 Getting Started

### Step 1: Get MAC Address
1. Open the `mac_scanner/` folder in Arduino IDE
2. Open `mac_scanner.ino` 
3. Upload the code to ESP8266
4. Open Serial Monitor (115200 baud)
5. Note the displayed MAC address

**Important:** The MAC scanner is in a separate folder because Arduino IDE doesn't allow multiple .ino files in the same folder!

### Step 2: Adjust Configuration
1. Open `outsideSensor.ino`
2. **IMPORTANT:** Change the following settings:

```cpp
// Unique sensor ID (1, 2, 3, ...)
#define SENSOR_ID 1

// MainStation MAC address (get from MainStation)
uint8_t main_station_mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
```

### Step 3: Configure MainStation
The ESP8266 MAC address must be registered in the MainStation:
```cpp
weather_station_register_remote_sensor(esp8266_mac_address, "Outside Sensor");
```

### Step 4: Upload Code
1. Select the correct COM port
2. Upload `outsideSensor.ino` to ESP8266
3. Open Serial Monitor for monitoring

## ⚙️ Configuration Options

### Adjust Send Interval
```cpp
#define SEND_INTERVAL 30000  // 30 seconds
```

### Enable Deep Sleep (Battery Operation)
```cpp
#define DEEP_SLEEP_ENABLED true
#define DEEP_SLEEP_MINUTES 5
```

### Change Sensor ID
```cpp
#define SENSOR_ID 2  // For second sensor
```

## 📊 Data Format

The sensor sends the following data structure:
```cpp
typedef struct {
    uint8_t sensor_id;          // Unique ID
    float temperature;          // Temperature in °C
    float humidity;             // Humidity in %
    uint32_t timestamp;         // Timestamp
    uint16_t battery_voltage;   // Battery voltage in mV
    bool valid;                 // Data valid
} esp_now_sensor_data_t;
```

## 🔍 Troubleshooting

### Problem: No data received
- ✅ MAC addresses entered correctly?
- ✅ Sensor registered in MainStation?
- ✅ ESP-NOW initialized on both devices?

### Problem: "ESP-NOW Init Failed"
- ✅ WiFi.mode(WIFI_STA) before esp_now_init()?
- ✅ Restart ESP8266

### Problem: DHT22 returns NaN
- ✅ Check wiring
- ✅ 5 second wait time after dht.begin()
- ✅ DHT22 vs DHT11 type correct?

### Problem: Short range
- ✅ Optimize antenna position
- ✅ Reduce obstacles
- ✅ Increase transmission power (if possible)

## 🔋 Battery Operation

For mobile operation:
1. Enable Deep Sleep: `#define DEEP_SLEEP_ENABLED true`
2. Increase send interval: e.g. every 5-10 minutes
3. Implement battery voltage monitoring
4. Use low-power hardware

## 📈 Advanced Features

### Measure Battery Voltage
```cpp
uint16_t read_battery_voltage() {
    uint16_t adc_value = analogRead(A0);
    return map(adc_value, 0, 1024, 0, 3300);
}
```

### Multiple Sensors
Each ESP8266 needs a unique `SENSOR_ID`:
- Sensor 1: `#define SENSOR_ID 1`
- Sensor 2: `#define SENSOR_ID 2`
- etc.

### Error Handling
The code includes comprehensive error handling:
- ESP-NOW initialization
- Sensor validation
- Send status monitoring

## 📝 Serial Monitor Output

Example output:
```
=== ESP8266 Remote Weather Sensor ===
Sensor ID: 1
DHT22 sensor initialized
ESP8266 MAC Address: AA:BB:CC:DD:EE:FF
✓ MainStation added as peer
=== Setup completed ===

📊 Reading sensor data...
🌡️  Temperature: 23.5°C
💧 Humidity: 65.2%
📡 Sending data to MainStation...
📤 Data sent
✓ Data sent successfully
---
```

## 🎯 Next Steps

1. Build and wire hardware
2. Get and enter MAC addresses
3. Adjust and upload code
4. Configure MainStation
5. Test and optimize system

Good luck with your ESP8266 weather sensor! 🌤️