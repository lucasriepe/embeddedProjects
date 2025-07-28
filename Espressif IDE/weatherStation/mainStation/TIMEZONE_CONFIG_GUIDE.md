# Timezone Configuration Guide

## Verwendung der Timezone-Konfiguration

Die neue Timezone-Konfiguration ermöglicht es Ihnen, die Zeitzone über das ESP-IDF Konfigurationssystem (`idf.py menuconfig`) zu konfigurieren.

## Konfiguration über menuconfig

1. Führen Sie `idf.py menuconfig` aus
2. Navigieren Sie zu "Time Configuration"
3. Wählen Sie eine der vordefinierten Zeitzonen oder "Custom"
4. Bei "Custom": Geben Sie Ihren eigenen POSIX Timezone-String ein
5. Konfigurieren Sie optional die NTP-Server

## Verfügbare Zeitzonen

### Vordefinierte Optionen:
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

**Beispiele:**
- `CET-1CEST,M3.5.0,M10.5.0/3` - Mitteleuropäische Zeit
- `EST5EDT,M3.2.0,M11.1.0` - US Eastern Time
- `JST-9` - Japan Standard Time (keine Sommerzeit)

## NTP Server Konfiguration

### Standard NTP Server:
- **Primary**: `pool.ntp.org`
- **Backup**: `time.nist.gov`

### Deutsche NTP Server (Beispiel):
- **Primary**: `de.pool.ntp.org`
- **Backup**: `ptbtime1.ptb.de`

## Implementierung im Code

Der Code verwendet automatisch die konfigurierten Werte:

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