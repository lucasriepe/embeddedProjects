# Timezone Configuration Guide

## Using the Timezone Configuration

The timezone configuration allows you to set the timezone through the ESP-IDF configuration system (`idf.py menuconfig`).

## Configuration via menuconfig

1. Run `idf.py menuconfig`
2. Navigate to "Time Configuration"
3. Select one of the predefined timezones or "Custom"
4. For "Custom": Enter your own POSIX timezone string
5. Optionally configure the NTP servers

## Available Timezones

### Predefined Options:
- **UTC**: Coordinated Universal Time (`UTC0`)
- **CET**: Central European Time (`CET-1CEST,M3.5.0,M10.5.0/3`)
- **EST**: Eastern Standard Time - US (`EST5EDT,M3.2.0,M11.1.0`)
- **PST**: Pacific Standard Time - US (`PST8PDT,M3.2.0,M11.1.0`)
- **JST**: Japan Standard Time (`JST-9`)
- **GMT**: Greenwich Mean Time (`GMT0`)
- **CST**: Central Standard Time - US (`CST6CDT,M3.2.0,M11.1.0`)
- **MST**: Mountain Standard Time - US (`MST7MDT,M3.2.0,M11.1.0`)

### Custom Timezone String Format (POSIX):
```
std offset [dst [offset] [,start[/time],end[/time]]]
```

**Examples:**
- `CET-1CEST,M3.5.0,M10.5.0/3` - Central European Time
- `EST5EDT,M3.2.0,M11.1.0` - US Eastern Time
- `JST-9` - Japan Standard Time (no daylight saving time)

## NTP Server Configuration

### Default NTP Servers:
- **Primary**: `pool.ntp.org`
- **Backup**: `time.nist.gov`

### Regional NTP Servers (Example):
- **Primary**: `de.pool.ntp.org`
- **Backup**: `ptbtime1.ptb.de`

## Implementation in Code

The code automatically uses the configured values:

```c
// Timezone wird automatisch aus CONFIG_TIMEZONE_POSIX gelesen
setenv("TZ", CONFIG_TIMEZONE_POSIX, 1);

// NTP Server aus Konfiguration
esp_sntp_setservername(0, CONFIG_NTP_SERVER);
esp_sntp_setservername(1, CONFIG_NTP_SERVER_BACKUP);
```

## Vorteile der neuen Konfiguration

1. **Benutzerfreundlich**: Einfache Auswahl über menuconfig
2. **Flexibel**: Unterstützt sowohl vordefinierte als auch custom Zeitzonen
3. **Konfigurierbare NTP Server**: Anpassung an lokale/bevorzugte Server
4. **Compile-Time Konfiguration**: Keine Laufzeit-Änderungen nötig
5. **Dokumentiert**: Hilfetext für alle Optionen verfügbar

## Migration von der alten Konfiguration

Die alte `wifi_config.h` Datei mit `TIMEZONE_STRING` wird nicht mehr benötigt. Alle Timezone-Einstellungen werden jetzt über das Kconfig-System verwaltet.