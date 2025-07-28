# HTTP Client Implementation für Outside Sensor

## Übersicht

Diese Implementierung ersetzt die ESP-NOW Kommunikation durch HTTP-basierte Datenabfrage für den Außensensor. Der HTTP-Client ruft Sensordaten von einem ESP32-S3 Server ab, der unter `http://192.168.178.59/dht` verfügbar ist.

## Implementierte Dateien

### 1. `http_client.h`
- Definiert die Datenstruktur `outside_sensor_data_t` für Außensensordaten
- Deklariert HTTP-Client Funktionen für Initialisierung, Datenabfrage und Cleanup

### 2. `http_client.c`
- Implementiert HTTP-Client mit ESP-IDF HTTP Client API
- Parst JSON-Response vom Format: `{'sensor': 'DHT', 'humI': 51.50, 'temp': 28.20, 'tmpIndex': 28.82}`
- Behandelt HTTP-Fehler und Netzwerkprobleme

### 3. Aktualisierte `weather_station_ui.c`
- Integriert HTTP-Client für Außensensordaten
- Aktualisiert UI alle 10 Sekunden mit neuen Daten
- Zeigt Verbindungsstatus (Connected/No WiFi/HTTP Error) an
- Behandelt Fehlerszenarien graceful

## Funktionsweise

1. **Initialisierung**: HTTP-Client wird beim Start der Weather Station initialisiert
2. **Datenabfrage**: Alle 10 Sekunden wird `http_client_fetch_sensor_data()` aufgerufen
3. **JSON Parsing**: Response wird geparst und in `outside_sensor_data_t` Struktur gespeichert
4. **UI Update**: Temperatur, Luftfeuchtigkeit und Status werden auf dem Display aktualisiert

## Konfiguration

- **Server URL**: `http://192.168.178.59/dht` (definiert in `OUTSIDE_SENSOR_URL`)
- **Update Intervall**: 10 Sekunden
- **Timeout**: 5 Sekunden für HTTP-Requests

## Fehlerbehandlung

- **Kein WiFi**: Zeigt "No WiFi" Status an
- **HTTP Fehler**: Zeigt "HTTP Error" Status an
- **JSON Parse Fehler**: Markiert Daten als ungültig
- **Timeout**: Automatischer Retry beim nächsten Update-Zyklus

## Dependencies

- `esp_http_client`: ESP-IDF HTTP Client Komponente
- `json`: ESP-IDF JSON Parser Komponente
- Beide wurden zu `CMakeLists.txt` hinzugefügt

## Status Anzeige

- 🟢 **Connected**: Erfolgreiche Datenabfrage
- 🔴 **No WiFi**: Keine WiFi-Verbindung
- 🔴 **HTTP Error**: HTTP-Request fehlgeschlagen

Die Implementierung ist robust und behandelt alle typischen Netzwerk- und Kommunikationsfehler graceful.