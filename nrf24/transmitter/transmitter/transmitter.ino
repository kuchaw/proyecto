#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <RF24.h>

// =========================
// WiFi + server
// =========================
const char* ssid = "Electronica";
const char* password = "KIRCHHOFF24";
const char* serverUrl = "https://cansat1.onrender.com/api/telemetry";

// =========================
// nRF24
// =========================
#define NRF_CE 25
#define NRF_CSN 26

RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "GAAY1";

// Must match transmitter struct exactly
struct TelemetryPacket {
  uint32_t counter;
  float lat;
  float lon;
  float alt;
  float temp;       // BME680 temperature
  float pressure;   // BME680 pressure
  float humidity;   // BME680 humidity
  uint8_t sat;
  uint8_t padding[3];
};

TelemetryPacket packet;

void connectWiFi();
void sendToServer(const TelemetryPacket& p);

void setup() {
  Serial.begin(115200);
  delay(1000);

  connectWiFi();

  SPI.begin(18, 19, 23, NRF_CSN);

  if (!radio.begin()) {
    Serial.println("nRF24 no detectado");
    while (1);
  }

  radio.openReadingPipe(0, address);
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.startListening();

  Serial.print("Expected packet size: ");
  Serial.println(sizeof(TelemetryPacket));

  Serial.println("Ground listo: nRF24 RX + HTTP POST");
}

void loop() {
  if (radio.available()) {
    radio.read(&packet, sizeof(packet));

    Serial.println("\n===== PACKET RECEIVED =====");

    Serial.print("Counter: ");
    Serial.println(packet.counter);

    Serial.print("Lat: ");
    Serial.println(packet.lat, 6);

    Serial.print("Lon: ");
    Serial.println(packet.lon, 6);

    Serial.print("Alt: ");
    Serial.println(packet.alt, 2);

    Serial.print("Sat: ");
    Serial.println(packet.sat);

    Serial.print("BME Temp: ");
    Serial.println(packet.temp, 2);

    Serial.print("BME Pressure: ");
    Serial.println(packet.pressure, 2);

    Serial.print("BME Humidity: ");
    Serial.println(packet.humidity, 2);

    sendToServer(packet);
  }
}

void connectWiFi() {
  Serial.print("Conectando a WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void sendToServer(const TelemetryPacket& p) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Reintentando...");
    connectWiFi();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("No se pudo reconectar WiFi");
      return;
    }
  }

  String json = "{";
  json += "\"lat\":" + String(p.lat, 6) + ",";
  json += "\"lon\":" + String(p.lon, 6) + ",";
  json += "\"alt\":" + String(p.alt, 2) + ",";
  json += "\"sat\":" + String(p.sat) + ",";
  json += "\"temp\":" + String(p.temp, 2) + ",";
  json += "\"pressure\":" + String(p.pressure, 2) + ",";
  json += "\"humidity\":" + String(p.humidity, 2);
  json += "}";

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, serverUrl);
  http.addHeader("Content-Type", "application/json");

  Serial.println("Enviando JSON:");
  Serial.println(json);

  int httpCode = http.POST(json);

  Serial.print("HTTP Response: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    Serial.println(http.getString());
  } else {
    Serial.print("Error HTTP: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}