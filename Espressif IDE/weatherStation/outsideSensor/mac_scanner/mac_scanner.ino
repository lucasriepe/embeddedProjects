/*
 * ESP8266 MAC Address Scanner
 * 
 * This program displays the MAC address of the ESP8266.
 * This MAC address must be registered in the MainStation.
 * 
 * Usage:
 * 1. Upload this code to the ESP8266
 * 2. Open Serial Monitor (115200 baud)
 * 3. Note the MAC address
 * 4. Enter MAC address in outsideSensor.ino AND MainStation
 */

#include <ESP8266WiFi.h>

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== ESP8266 MAC Address Scanner ===");
    
    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    
    // Display MAC address
    String mac = WiFi.macAddress();
    Serial.println("📍 MAC Address of this ESP8266:");
    Serial.println(mac);
    
    // Formatted for C code
    Serial.println("\n📋 For C code (Array format):");
    Serial.print("uint8_t mac[] = {");
    
    // Split MAC into bytes
    for (int i = 0; i < 6; i++) {
        String hex = mac.substring(i*3, i*3+2);
        Serial.print("0x");
        Serial.print(hex);
        if (i < 5) Serial.print(", ");
    }
    Serial.println("};");
    
    Serial.println("\n✅ Copy this MAC address to:");
    Serial.println("   1. outsideSensor.ino → main_station_mac[]");
    Serial.println("   2. MainStation → weather_station_register_remote_sensor()");
    
    Serial.println("\n🔄 Restart in 10 seconds...");
}

void loop() {
    delay(10000);
    ESP.restart();
}