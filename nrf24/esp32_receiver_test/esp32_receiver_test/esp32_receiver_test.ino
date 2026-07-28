#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <RF24.h>

// =========================
// WiFi + server
// =========================
const char* ssid = "sf2026";
const char* password = "16111505";
const char* serverUrl = "https://cansat1.onrender.com/api/telemetry";

// =========================
// nRF24
// =========================
#define NRF_CE 4
#define NRF_CSN 5

RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "GAAY1";

struct TelemetryPacket {
  uint32_t counter;
  float lat;
  float lon;
  float alt;
  float temp;
  float pressure;
  float humidity;
  uint8_t sat;
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
    Serial.println(packet.alt);

    Serial.print("Sat: ");
    Serial.println(packet.sat);

    Serial.print("Temp: ");
    Serial.println(packet.temp);

    Serial.print("Pressure: ");
    Serial.println(packet.pressure);

    Serial.print("Humidity: ");
    Serial.println(packet.humidity);

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
    if (WiFi.status() != WL_CONNECTED) return;
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