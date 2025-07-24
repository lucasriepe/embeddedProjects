#ifdef ESP8266 
   #include <ESP8266WiFi.h> 
   #include <espnow.h> 
 #else 
   #error "This project is for ESP8266 only! Select ESP8266 Board in Arduino IDE." 
 #endif 
 
 #include <DHT.h> 
 
 // ========== CONFIGURATION ========== 
 #define SENSOR_ID 1                    // UNIQUE ID for this sensor (1, 2, 3, ...) 
 #define DHTPIN D2                      // D2 pin on NodeMCU (GPIO4)
 #define DHTTYPE DHT22                  // DHT22 Sensor (requires 5V!) 
 #define SEND_INTERVAL 30000            // Send interval in ms (30 seconds) 
 #define DEEP_SLEEP_ENABLED false       // true = Enable Deep Sleep (battery operation) 
 #define DEEP_SLEEP_MINUTES 5           // Deep Sleep duration in minutes 
 
 // IMPORTANT: Connect DHT22 to VIN (5V), not to 3.3V! 
 // If readings are weak or unreliable → check power supply 
 
 // MainStation MAC Address - CORRECT ADDRESS! 
 uint8_t main_station_mac[] = {0xF0, 0xF5, 0xBD, 0x4B, 0x9C, 0x8C}; 
 
 // ========== DATA STRUCTURE ========== 
 // EXACTLY as defined in MainStation! 
 typedef struct { 
     uint8_t sensor_id;          // Unique sensor ID 
     float temperature;          // Temperature in Celsius 
     float humidity;             // Humidity in % 
     uint32_t timestamp;         // Timestamp 
     uint16_t battery_voltage;   // Battery voltage in mV (0 for mains-powered) 
     bool valid;                 // Data valid 
 } esp_now_sensor_data_t; 
 
 // ========== GLOBAL VARIABLES ========== 
 DHT dht(DHTPIN, DHTTYPE); 
 unsigned long last_send_time = 0; 
 bool esp_now_ready = false; 
 
 // ========== ESP-NOW CALLBACK ========== 
 void on_data_sent(uint8_t *mac_addr, uint8_t sendStatus) { 
     Serial.printf("📡 Send to: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                   mac_addr[0], mac_addr[1], mac_addr[2], 
                   mac_addr[3], mac_addr[4], mac_addr[5]);
     if (sendStatus == 0) { 
         Serial.println("✓ Data sent successfully"); 
     } else { 
         Serial.printf("✗ Error sending data (Status: %d)\n", sendStatus); 
         Serial.println("💡 Possible causes:");
         Serial.println("   - Wrong MAC address");
         Serial.println("   - Different WiFi channel");
         Serial.println("   - MainStation not ready");
         Serial.println("   - Distance too far");
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
     
     // Set WiFi channel to 11 (same as MainStation)
     wifi_set_channel(11); 
     
     // Display own MAC address 
     Serial.print("ESP8266 MAC Address: "); 
     Serial.println(WiFi.macAddress()); 
     
     // Display WiFi channel
     Serial.printf("WiFi Channel: %d\n", wifi_get_channel());
     
     // Display MainStation MAC
     Serial.printf("MainStation MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   main_station_mac[0], main_station_mac[1], main_station_mac[2],
                   main_station_mac[3], main_station_mac[4], main_station_mac[5]); 
     
     // Initialize ESP-NOW 
     if (esp_now_init() != 0) { 
         Serial.println("✗ ESP-NOW initialization failed!"); 
         return; 
     } 
     
     // Register callback for send status 
     esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER); 
     esp_now_register_send_cb(on_data_sent); 
     
     // Add MainStation as peer 
     int result = esp_now_add_peer(main_station_mac, ESP_NOW_ROLE_SLAVE, 11, NULL, 0); 
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
     
     // Create data packet 
     esp_now_sensor_data_t sensor_data = { 
         .sensor_id = SENSOR_ID, 
         .temperature = temperature, 
         .humidity = humidity, 
         .timestamp = millis(), 
         .battery_voltage = 0,        // 0 = mains-powered (no battery) 
         .valid = data_valid 
     }; 
     
     // Send data 
     Serial.println("📡 Sending data to MainStation..."); 
     int result = esp_now_send(main_station_mac, (uint8_t*)&sensor_data, sizeof(sensor_data)); 
     
     if (result == 0) { 
         Serial.println("📤 Send command successful"); 
     } else { 
         Serial.printf("❌ Send failed: %d\n", result); 
     } 
     
     Serial.println("----------------------------------------\n"); 
 }