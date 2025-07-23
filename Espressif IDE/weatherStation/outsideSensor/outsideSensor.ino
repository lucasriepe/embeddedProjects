/*
 * ESP8266 Remote Weather Sensor
 * Sends temperature and humidity data via ESP-NOW to MainStation
 * 
 * IMPORTANT: This project requires Arduino IDE with ESP8266 Board Package!
 * 
 * Arduino IDE Setup:
 * 1. File → Preferences → Additional Board Manager URLs:
 *    http://arduino.esp8266.com/stable/package_esp8266com_index.json
 * 2. Tools → Board → Boards Manager → Search "ESP8266" → Install
 * 3. Tools → Board → ESP8266 Boards → NodeMCU 1.0 (ESP-12E Module)
 * 4. Tools → Manage Libraries → Search "DHT sensor library" → Install Adafruit Version
 * 
 * Hardware:
 * - ESP8266 (NodeMCU, Wemos D1 Mini, etc.)
 * - DHT22 Sensor: VCC→5V (VIN), GND→GND, DATA→GPIO2 (D4)
 *   IMPORTANT: DHT22 requires 5V! Use VIN pin on ESP8266
 * 
 * DHT22 Wiring:
 * DHT22 VCC  → ESP8266 VIN (5V)
 * DHT22 GND  → ESP8266 GND
 * DHT22 DATA → ESP8266 GPIO2 (D4)
 * 
 * Note: GPIO2 is 5V-tolerant, so data line works fine
 * 
 * Libraries:
 * - ESP8266WiFi (included in ESP8266 Board Package)
 * - espnow (included in ESP8266 Board Package)
 * - DHT sensor library (Adafruit - install separately)
 */

// ESP8266-specific includes (only available with ESP8266 Board Package)
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <espnow.h>
#else
  #error "This project is for ESP8266 only! Select ESP8266 Board in Arduino IDE."
#endif

#include <DHT.h>

// ========== CONFIGURATION ==========
#define SENSOR_ID 1                    // UNIQUE ID for this sensor (1, 2, 3, ...)
#define DHT_PIN 2                      // GPIO2 (D4 on NodeMCU)
#define DHT_TYPE DHT22                 // DHT22 Sensor (requires 5V!)
#define SEND_INTERVAL 30000            // Send interval in ms (30 seconds)
#define DEEP_SLEEP_ENABLED false       // true = Enable Deep Sleep (battery operation)
#define DEEP_SLEEP_MINUTES 5           // Deep Sleep duration in minutes

// IMPORTANT: Connect DHT22 to VIN (5V), not to 3.3V!
// If readings are weak or unreliable → check power supply

// MainStation MAC Address - MUST BE ADJUSTED!
// Get MAC with separate MAC-Scanner sketch
uint8_t main_station_mac[] = {0xF0, 0xF5, 0xBD, 0x4B, 0x9C, 0x8C};

// ========== DATA STRUCTURE ==========
// EXACTLY as defined in MainStation!
typedef struct {
    uint8_t sensor_id;          // Unique sensor ID
    float temperature;          // Temperature in Celsius
    float humidity;             // Humidity in %
    uint32_t timestamp;         // Timestamp
    uint16_t battery_voltage;   // Battery voltage in mV
    bool valid;                 // Data valid
} esp_now_sensor_data_t;

// ========== GLOBAL VARIABLES ==========
DHT dht(DHT_PIN, DHT_TYPE);
unsigned long last_send_time = 0;
bool esp_now_ready = false;

// ========== ESP-NOW CALLBACK ==========
void on_data_sent(uint8_t *mac_addr, uint8_t sendStatus) {
    if (sendStatus == 0) {
        Serial.println("✓ Data sent successfully");
    } else {
        Serial.println("✗ Error sending data");
    }
}

// ========== SETUP ==========
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP8266 Remote Weather Sensor ===");
    Serial.printf("Sensor ID: %d\n", SENSOR_ID);
    
    // Initialize DHT sensor
    dht.begin();
    Serial.println("DHT22 sensor initialized");
    
    // Configure WiFi for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Display own MAC address
    Serial.print("ESP8266 MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("✗ ESP-NOW initialization failed!");
        return;
    }
    
    // Register callback for send status
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(on_data_sent);
    
    // Add MainStation as peer
    int result = esp_now_add_peer(main_station_mac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    if (result == 0) {
        Serial.println("✓ MainStation added as peer");
        esp_now_ready = true;
    } else {
        Serial.printf("✗ Error adding MainStation: %d\n", result);
    }
    
    Serial.println("=== Setup completed ===\n");
    
    // First measurement after 5 seconds
    delay(5000);
}

// ========== MAIN LOOP ==========
void loop() {
    unsigned long current_time = millis();
    
    // Check if send time reached
    if (current_time - last_send_time >= SEND_INTERVAL) {
        send_sensor_data();
        last_send_time = current_time;
        
        // Enable Deep Sleep (only for battery operation)
        if (DEEP_SLEEP_ENABLED) {
            Serial.printf("💤 Deep Sleep for %d minutes...\n", DEEP_SLEEP_MINUTES);
            delay(100); // Short wait for Serial output
            ESP.deepSleep(DEEP_SLEEP_MINUTES * 60 * 1000000UL); // Microseconds
        }
    }
    
    // Short pause
    delay(1000);
}

// ========== SEND SENSOR DATA ==========
void send_sensor_data() {
    if (!esp_now_ready) {
        Serial.println("✗ ESP-NOW not ready");
        return;
    }
    
    Serial.println("📊 Reading sensor data...");
    
    // Read sensor data
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // Validate data
    bool data_valid = !isnan(temperature) && !isnan(humidity);
    
    if (data_valid) {
        Serial.printf("🌡️  Temperature: %.1f°C\n", temperature);
        Serial.printf("💧 Humidity: %.1f%%\n", humidity);
    } else {
        Serial.println("⚠️  Invalid sensor data!");
        temperature = 0.0;
        humidity = 0.0;
    }
    
    // Read battery voltage (optional)
    uint16_t battery_voltage = read_battery_voltage();
    
    // Create data packet
    esp_now_sensor_data_t sensor_data = {
        .sensor_id = SENSOR_ID,
        .temperature = temperature,
        .humidity = humidity,
        .timestamp = millis(),
        .battery_voltage = battery_voltage,
        .valid = data_valid
    };
    
    // Send data
    Serial.println("📡 Sending data to MainStation...");
    int result = esp_now_send(main_station_mac, (uint8_t*)&sensor_data, sizeof(sensor_data));
    
    if (result == 0) {
        Serial.println("📤 Data sent");
    } else {
        Serial.printf("✗ Send error: %d\n", result);
    }
    
    Serial.println("---");
}

// ========== READ BATTERY VOLTAGE ==========
uint16_t read_battery_voltage() {
    // For ESP8266: ADC reads VCC (with appropriate configuration)
    // Alternative: Voltage divider on A0
    
    // Simple method: Return fixed voltage
    return 3300; // 3.3V in mV
    
    // Advanced method (requires hardware modification):
    // uint16_t adc_value = analogRead(A0);
    // return map(adc_value, 0, 1024, 0, 3300);
}

// ========== HELPER FUNCTIONS ==========
void print_mac_address(uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
}