#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "DHT.h"

#define DHTPIN D2

#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

#ifndef STASSID
#define STASSID "FRITZ!Box 7590 NM"
#define STAPSK "95065973249267578128"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

ESP8266WebServer server(80);

void handleRoot() {
  server.send(200, "text/html", "hello from esp8266 and lucas :D\r\n");
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) { message += " " + server.argName(i) + ": " + server.arg(i) + "\n"; }
  server.send(404, "text/plain", message);
}

void handleDHT() {
  float hDht = dht.readHumidity();
  float tDht = dht.readTemperature();

  if (isnan(hDht) || isnan(tDht)) {
    String errorJson = "{\"error\": \"DHT sensor failed to read data\", \"status\": \"error\"}";
    server.send(500, "application/json", errorJson);
    return;
  }

  float hicDht = dht.computeHeatIndex(tDht, hDht, false);

  String json = "{\"sensor\": \"DHT\", \"humidity\": " + String(hDht, 2) + ", \"temperature\": " + String(tDht, 2) + ", \"heatIndex\": " + String(hicDht, 2) + ", \"status\": \"success\"}";

  server.send(200, "application/json", json);
}

void setup(void) {
  Serial.begin(9600);
  dht.begin();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  pinMode(LED_BUILTIN, OUTPUT);


  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
  }

  //Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/dht", handleDHT);

/*
  server.on("/inline", []() {
    server.send(200, "text/html", "this works as well");
  });
*/

  server.onNotFound(handleNotFound);

  /////////////////////////////////////////////////////////
  // Hook examples
  /*
  server.addHook([](const String& method, const String& url, WiFiClient* client, ESP8266WebServer::ContentTypeFunction contentType) {
    (void)method;       // GET, PUT, ...
    (void)url;          // example: /root/myfile.html
    (void)client;       // the webserver tcp client connection
    (void)contentType;  // contentType(".html") => "text/html"
    Serial.printf("A useless web hook has passed\n");
    Serial.printf("(this hook is in 0x%08x area (401x=IRAM 402x=FLASH))\n", esp_get_program_counter());
    return ESP8266WebServer::CLIENT_REQUEST_CAN_CONTINUE;
  });
  */
  /////////////////////////////////////////////////////////////

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();
  MDNS.update();
}
