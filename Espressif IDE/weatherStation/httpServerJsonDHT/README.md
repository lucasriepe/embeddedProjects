# ESP8266 Weather Station with DHT22 Sensor

A simple HTTP server running on ESP8266 that provides temperature and humidity readings from a DHT22 sensor via JSON API.

## Features

- 📡 WiFi connectivity
- 🌡️ Temperature and humidity monitoring using DHT22 sensor
- 🔗 RESTful JSON API endpoint
- 💡 Built-in LED status indicator
- 🔄 Real-time sensor data via HTTP requests

## Hardware Requirements

- **ESP8266 Development Board** (NodeMCU, Wemos D1 Mini, etc.)
- **DHT22 Temperature & Humidity Sensor**
- **Jumper wires**
- **Breadboard** (optional)

## Wiring Diagram

Connect the DHT22 sensor to your ESP8266 as follows:

| DHT22 Pin | ESP8266 Pin | Description |
|-----------|-------------|-------------|
| VCC       | 5V          | Power supply (5V) |
| GND       | GND         | Ground |
| DATA      | D2          | Data pin (GPIO4) |

### Wiring Instructions

1. **Power (VCC)**: Connect the DHT22 VCC pin to the 5V pin on your ESP8266
2. **Ground (GND)**: Connect the DHT22 GND pin to any GND pin on your ESP8266
3. **Data (DATA)**: Connect the DHT22 DATA pin to pin D2 (GPIO4) on your ESP8266

> **Note**: Some DHT22 modules may work with 3.3V, but 5V is recommended for stable operation.

## Software Setup

### Prerequisites

- Arduino IDE installed
- ESP8266 board package installed in Arduino IDE
- Required libraries (see below)

### Required Libraries

Install the following libraries through Arduino IDE Library Manager:

1. **ESP8266WiFi** (usually included with ESP8266 board package)
2. **ESP8266WebServer** (usually included with ESP8266 board package)
3. **ESP8266mDNS** (usually included with ESP8266 board package)
4. **DHT sensor library** by Adafruit

### Installation Steps

1. **Clone or download** this repository
2. **Open** `httpServerJsonDHT.ino` in Arduino IDE
3. **Configure WiFi credentials**:
   ```cpp
   #define STASSID "Your_WiFi_Network_Name"
   #define STAPSK "Your_WiFi_Password"
   ```
4. **Select your ESP8266 board** in Arduino IDE:
   - Tools → Board → ESP8266 Boards → (Select your board)
5. **Select the correct port**:
   - Tools → Port → (Select your ESP8266 port)
6. **Upload** the code to your ESP8266

## Usage

### Initial Setup

1. After uploading the code, the ESP8266 will attempt to connect to your WiFi network
2. The built-in LED will blink during connection process
3. Once connected, the LED will stop blinking
4. Open Serial Monitor (9600 baud) to see the connection status

### API Endpoints

#### Get Sensor Data
```
GET /dht
```

**Success Response:**
```json
{
  "sensor": "DHT",
  "humidity": 65.20,
  "temperature": 23.50,
  "heatIndex": 24.15,
  "status": "success"
}
```

**Error Response:**
```json
{
  "error": "DHT sensor failed to read data",
  "status": "error"
}
```

#### Root Endpoint
```
GET /
```
Returns a simple greeting message.

### Finding Your ESP8266 IP Address

1. Open Serial Monitor in Arduino IDE
2. Reset your ESP8266
3. Look for the IP address in the serial output
4. Access your weather station at: `http://[IP_ADDRESS]/dht`

### Example Usage

```bash
# Get sensor data
curl http://192.168.1.100/dht

# Example response
{
  "sensor": "DHT",
  "humidity": 45.30,
  "temperature": 22.10,
  "heatIndex": 21.85,
  "status": "success"
}
```

## Troubleshooting

### Common Issues

1. **DHT sensor not responding**:
   - Check wiring connections
   - Ensure DHT22 is getting 5V power
   - Verify DATA pin is connected to D2

2. **WiFi connection fails**:
   - Double-check WiFi credentials
   - Ensure WiFi network is 2.4GHz (ESP8266 doesn't support 5GHz)
   - Check signal strength

3. **Compilation errors**:
   - Install required libraries
   - Select correct ESP8266 board
   - Update ESP8266 board package

### LED Status Indicators

- **Blinking**: Connecting to WiFi
- **Solid OFF**: Connected successfully
- **Solid ON**: Connection failed or other error

## Technical Details

- **Microcontroller**: ESP8266
- **Sensor**: DHT22 (AM2302)
- **Communication**: HTTP REST API
- **Data Format**: JSON
- **WiFi**: 2.4GHz 802.11 b/g/n
- **Power**: 5V via USB or external supply

## License

This project is open source and available under the MIT License.

## Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.