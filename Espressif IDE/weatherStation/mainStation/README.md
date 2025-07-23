# ESP32-S3 Weather Station

A modern weather station implementation using ESP32-S3 with LVGL GUI, WiFi connectivity, and time synchronization.

## Features

- **Modern GUI**: Clean interface with green header bar and time display
- **WiFi Connectivity**: Automatic connection with reconnection logic
- **Time Display**: German time (CET/CEST) with automatic daylight saving time
- **Secure Configuration**: WiFi credentials stored in `.env` file (not committed to git)
- **Custom Hostname**: Device appears as "ESPmainStation" on the network

## Hardware Requirements

- ESP32-S3 development board
- Compatible LCD display with touch support
- Waveshare RGB LCD (as configured in this project)

## Software Requirements

- ESP-IDF (Espressif IoT Development Framework)
- Python 3.x
- Git

## Project Setup

### 1. Clone the Repository

```bash
git clone https://github.com/lucasriepe/embeddedProjects/tree/6ea664f73fad7934fd76d3475e34dc2169799aa6/Espressif%20IDE/weatherStation/mainStation
cd weatherStation/mainStation
```

### 2. Install ESP-IDF

Follow the [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html) to install ESP-IDF for your operating system.

### 3. Configure WiFi Credentials

1. Copy the example environment file:

   ```bash
   cp .env.example .env
   ```

2. Edit the `.env` file with your WiFi credentials and timezone:

   ```
   # WiFi Configuration
   WIFI_SSID=YourWiFiNetworkName
   WIFI_PASSWORD=YourWiFiPassword
   
   # Timezone Configuration
   # Format: TZ environment variable format
   # Examples:
   # Europe/Berlin (Germany): CET-1CEST,M3.5.0,M10.5.0/3
   # America/New_York (USA East): EST5EDT,M3.2.0,M11.1.0
   # America/Los_Angeles (USA West): PST8PDT,M3.2.0,M11.1.0
   # Asia/Tokyo (Japan): JST-9
   # UTC: UTC0
   TIMEZONE=CET-1CEST,M3.5.0,M10.5.0/3
   ```

3. Generate the WiFi configuration header:
   ```bash
   python3 generate_wifi_config.py
   ```

   This will create `main/wifi_config.h` with your WiFi credentials and timezone settings.

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
├── .env                    # WiFi credentials (not in git)
├── .env.example           # Example environment file
├── .gitignore             # Git ignore rules
├── CMakeLists.txt         # Main CMake configuration
├── README.md              # This file
├── components/            # External components
│   ├── espressif__esp_lcd_touch/
│   ├── espressif__esp_lcd_touch_gt911/
│   └── lvgl__lvgl/
├── generate_wifi_config.py # Script to generate WiFi config
├── main/                  # Main application code
│   ├── CMakeLists.txt
│   ├── main.c             # Application entry point
│   ├── weather_station_ui.c # Main UI and WiFi logic
│   ├── wifi_config.h      # Generated WiFi configuration
│   ├── lvgl_port.c        # LVGL porting layer
│   └── waveshare_rgb_lcd_port.c # LCD driver
├── partitions.csv         # Custom partition table
├── sdkconfig              # ESP-IDF configuration
└── sdkconfig.defaults     # Default configuration
```

## Configuration

### WiFi Configuration

The project uses a secure configuration system:

1. WiFi credentials are stored in `.env` file (excluded from git)
2. Run `python3 generate_wifi_config.py` to generate `main/wifi_config.h`
3. The generated header file contains the WiFi credentials for compilation

### Time Configuration

The device supports configurable timezones through the `TIMEZONE` variable in the `.env` file. The timezone format follows the POSIX TZ environment variable format.

#### Common Timezone Examples:
- **Germany (CET/CEST)**: `CET-1CEST,M3.5.0,M10.5.0/3`
- **USA East Coast (EST/EDT)**: `EST5EDT,M3.2.0,M11.1.0`
- **USA West Coast (PST/PDT)**: `PST8PDT,M3.2.0,M11.1.0`
- **Japan (JST)**: `JST-9`
- **UTC**: `UTC0`
- **UK (GMT/BST)**: `GMT0BST,M3.5.0,M10.5.0/3`
- **Australia Sydney (AEST/AEDT)**: `AEST-10AEDT,M10.1.0,M4.1.0/3`

#### Timezone Format Explanation:
The format is: `STD[offset]DST[offset],start[/time],end[/time]`
- **STD**: Standard time zone abbreviation
- **offset**: Hours offset from UTC (negative for east of UTC)
- **DST**: Daylight saving time abbreviation (optional)
- **start**: When DST starts (M = month, week, day)
- **end**: When DST ends

Example: `CET-1CEST,M3.5.0,M10.5.0/3`
- CET: Central European Time
- -1: 1 hour ahead of UTC
- CEST: Central European Summer Time
- M3.5.0: DST starts on the last (5th) Sunday (0) of March (3)
- M10.5.0/3: DST ends on the last Sunday of October at 3 AM

- **NTP Servers**: `pool.ntp.org` and `de.pool.ntp.org`
- **Display Format**: HH:MM (24-hour format without seconds)

### Display Configuration

- **Background**: Light gray (#F5F5F5)
- **Header Bar**: Green (#4CAF50) with "Weatherstation" title
- **Time Display**: Black text in top-right corner
- **Font**: Montserrat (18pt for time, 20pt for title)

## Troubleshooting

### Build Issues

1. **Partition too small error**: The project includes a custom partition table with 2MB app partition
2. **Missing components**: Run `idf.py reconfigure` to download missing components
3. **SNTP errors**: Ensure you're using the correct ESP-IDF version (v5.x recommended)

### WiFi Issues

1. **Connection failed**: Check WiFi credentials in `.env` file
2. **Hostname not visible**: Some routers may take time to update device names
3. **Time not syncing**: Ensure internet connectivity and check NTP server accessibility

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

- Never commit `.env` file to version control
- Use strong WiFi passwords
- Consider implementing OTA updates for production use
- Validate all external data inputs

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
