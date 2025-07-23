# MAC Address Scanner for ESP8266

This utility helps you find the MAC address of your ESP8266, which is needed for ESP-NOW communication.

## 📋 Usage

1. **Open in Arduino IDE:**
   - Open the `mac_scanner.ino` file in Arduino IDE
   - Make sure you have the ESP8266 board package installed

2. **Upload to ESP8266:**
   - Select your ESP8266 board (e.g., NodeMCU 1.0)
   - Select the correct COM port
   - Upload the sketch

3. **View MAC Address:**
   - Open Serial Monitor (Tools → Serial Monitor)
   - Set baud rate to 115200
   - The MAC address will be displayed

4. **Copy MAC Address:**
   - Note down the displayed MAC address
   - Use this address in your main `outsideSensor.ino` configuration
   - Also register this MAC address in your MainStation

## 📝 Example Output

```
=== ESP8266 MAC Address Scanner ===
ESP8266 MAC Address: AA:BB:CC:DD:EE:FF
Copy this MAC address for configuration!
```

## 🔄 Next Steps

After getting the MAC address:

1. **Configure outsideSensor.ino:**
   - Open the main `outsideSensor.ino` file
   - Update the MainStation MAC address
   - Set your unique sensor ID

2. **Register in MainStation:**
   - Add this ESP8266's MAC address to your MainStation
   - Use a descriptive name like "Outside Sensor"

3. **Upload Main Code:**
   - Upload the configured `outsideSensor.ino`
   - Start monitoring weather data!

## ⚠️ Important Notes

- Each ESP8266 has a unique MAC address
- This address is needed for ESP-NOW peer registration
- Keep this address safe for future reference
- The MAC address doesn't change unless you replace the ESP8266

---

**Return to main project:** Go back to the parent folder and open `outsideSensor.ino` for the main weather sensor code.