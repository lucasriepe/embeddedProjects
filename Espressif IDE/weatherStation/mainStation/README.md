# ESP32-S3 Weather Station

A modern weather station implementation using ESP32-S3 with LVGL GUI, WiFi connectivity, and time synchronization.

## Features

- **Modern GUI**: Clean interface with weather data display
- **WiFi Connectivity**: Automatic connection with configurable retry logic
- **Time Display**: Configurable timezone with automatic daylight saving time
- **Sensor Data**: Inside DHT22 sensor and outside HTTP sensor support
- **Secure Configuration**: All settings configured via ESP-IDF menuconfig
- **Custom Hostname**: Configurable device hostname for network identification

## Hardware Requirements

- ESP32-S3 development board
- Compatible LCD display with touch support
- Waveshare RGB LCD (as configured in this project)
- DHT22 temperature/humidity sensor

## Software Requirements

- ESP-IDF (Espressif IoT Development Framework) v5.x
- Git

## Project Setup

### 1. Clone the Repository

```bash
git clone https://github.com/lucasriepe/embeddedProjects/tree/6ea664f73fad7934fd76d3475e34dc2169799aa6/Espressif%20IDE/weatherStation/mainStation
cd weatherStation/mainStation
```

### 2. Install ESP-IDF

Follow the [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html) to install ESP-IDF for your operating system.

### 3. Configure Project Settings

Configure all project settings using the ESP-IDF configuration system:

```bash
idf.py menuconfig
```

Navigate to **Weather Station Configuration** to configure:

#### WiFi Configuration

- WiFi SSID and Password
- Device Hostname
- Maximum Retry Attempts
- Scan Method and Power Save Settings
- HTTP Client Timeout
- Outside Sensor URL
- Sensor Update Interval

#### Time Configuration

- Timezone (predefined options or custom POSIX string)
- NTP Servers (primary and backup)

For detailed configuration instructions, see:

- [WiFi Configuration Guide](WIFI_CONFIG_GUIDE.md)
- [Timezone Configuration Guide](TIMEZONE_CONFIG_GUIDE.md)

### 4. Build the Project

1. Set up the ESP-IDF environment:

   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

2. Configure the project (if needed):

   ```bash
   idf.py menuconfig
   ```

3. Build the project:
   ```bash
   idf.py build
   ```

### 5. Flash and Monitor

1. Connect your ESP32-S3 board via USB

2. Flash the firmware:

   ```bash
   idf.py flash
   ```

3. Monitor the output:

   ```bash
   idf.py monitor
   ```

   Press `Ctrl+]` to exit the monitor.

## Project Structure

```
mainStation/
├── .gitignore             # Git ignore rules
├── CMakeLists.txt         # Main CMake configuration
├── README.md              # This file
├── WIFI_CONFIG_GUIDE.md   # WiFi configuration guide
├── TIMEZONE_CONFIG_GUIDE.md # Timezone configuration guide
├── HTTP_CLIENT_README.md  # HTTP client documentation
├── components/            # External components
│   ├── espressif__esp_lcd_touch/
│   ├── espressif__esp_lcd_touch_gt911/
│   └── lvgl__lvgl/
├── main/                  # Main application code
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild  # Project configuration options
│   ├── main.c             # Application entry point
│   ├── weather_station_ui.c # Main UI and WiFi logic
│   ├── weather_station_ui.h # UI header file
│   ├── dht22_sensor.c     # DHT22 sensor driver
│   ├── dht22_sensor.h     # DHT22 sensor header
│   ├── http_client.c      # HTTP client implementation
│   ├── http_client.h      # HTTP client header
│   ├── lvgl_port.c        # LVGL porting layer
│   └── waveshare_rgb_lcd_port.c # LCD driver
├── partitions.csv         # Custom partition table
├── sdkconfig              # ESP-IDF configuration
└── sdkconfig.defaults     # Default configuration
```

## Configuration

### WiFi Configuration

The project uses the ESP-IDF configuration system for all settings:

1. All WiFi settings are configured via `idf.py menuconfig`
2. Navigate to **Weather Station Configuration** → **WiFi Configuration**
3. Configure SSID, password, hostname, retry attempts, and power settings
4. Settings are stored in `sdkconfig` and compiled into the firmware

For detailed WiFi configuration options, see [WIFI_CONFIG_GUIDE.md](WIFI_CONFIG_GUIDE.md).

### Time Configuration

The device supports configurable timezones through the ESP-IDF configuration system:

1. Use `idf.py menuconfig` to access **Weather Station Configuration** → **Time Configuration**
2. Choose from predefined timezones or enter a custom POSIX string
3. Configure primary and backup NTP servers

For detailed timezone configuration, see [TIMEZONE_CONFIG_GUIDE.md](TIMEZONE_CONFIG_GUIDE.md).

#### Common Timezone Examples:

- **Germany (CET/CEST)**: `CET-1CEST,M3.5.0,M10.5.0/3`
- **USA East Coast (EST/EDT)**: `EST5EDT,M3.2.0,M11.1.0`
- **USA West Coast (PST/PDT)**: `PST8PDT,M3.2.0,M11.1.0`
- **Japan (JST)**: `JST-9`
- **UTC**: `UTC0`
- **UK (GMT/BST)**: `GMT0BST,M3.5.0,M10.5.0/3`
- **Australia Sydney (AEST/AEDT)**: `AEST-10AEDT,M10.1.0,M4.1.0/3`

### Sensor Configuration

- **Inside Sensor**: DHT22 connected to configurable GPIO pin
- **Outside Sensor**: HTTP client fetches data from configurable URL
- **Update Interval**: Configurable sensor reading interval
- **HTTP Timeout**: Configurable timeout for HTTP requests

### Display Configuration

- **Background**: White background with colored sensor boxes
- **Time Display**: Configurable timezone with HH:MM format
- **Sensor Boxes**: Inside and Outside sensor data with status indicators
- **Font**: Montserrat font family for clean appearance

## Troubleshooting

### Build Issues

1. **Partition too small error**: The project includes a custom partition table with 2MB app partition
2. **Missing components**: Run `idf.py reconfigure` to download missing components
3. **SNTP errors**: Ensure you're using the correct ESP-IDF version (v5.x recommended)

### WiFi Issues

1. **Connection failed**: Check WiFi credentials in menuconfig
2. **Hostname not visible**: Some routers may take time to update device names
3. **Time not syncing**: Ensure internet connectivity and check NTP server accessibility
4. **Maximum retries reached**: Increase the retry count in menuconfig

### Sensor Issues

1. **Inside sensor error**: Check DHT22 wiring and power
2. **Outside sensor error**: Verify the outside sensor URL and HTTP timeout settings
3. **"HTTP Error" message**: Check WiFi connectivity and outside sensor availability

### Display Issues

1. **Blank screen**: Check LCD connections and power supply
2. **Touch not working**: Verify touch controller configuration
3. **Wrong colors**: Check RGB LCD pin configuration

## Development

### Adding New Features

1. **Weather Data**: Extend the UI to display weather information from APIs
2. **Sensors**: Add temperature, humidity, or pressure sensors
3. **Data Logging**: Implement data storage and historical views
4. **Web Interface**: Add a web server for remote monitoring

### Code Style

- Use English for all comments and documentation
- Follow ESP-IDF coding conventions
- Keep functions modular and well-documented
- Use meaningful variable and function names

## Security Notes

- WiFi credentials are stored securely in the ESP-IDF configuration system
- Use strong WiFi passwords
- Consider implementing OTA updates for production use
- Validate all external data inputs
- Review HTTP client security settings for production deployments

## License

This project is licensed under the Apache License 2.0. See the source files for detailed license information.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes with appropriate tests
4. Submit a pull request with a clear description

## Support

For issues and questions:

1. Check the troubleshooting section above
2. Review ESP-IDF documentation
3. Check LVGL documentation for UI-related issues
4. Create an issue in the repository with detailed information
