#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <RF24.h>

// =========================
// WiFi + server
// =========================
const char* ssid = "";
const char* password = "";

const char* serverUrl =
  "https://cansat1.onrender.com/api/telemetry";

// =========================
// nRF24
// =========================
#define NRF_CE 25
#define NRF_CSN 26

RF24 radio(NRF_CE, NRF_CSN);

const byte address[6] = "GAAY1";

// Must match modos_de_vuelo.ino exactly
struct TelemetryPacket {
  uint32_t counter;
  float lat;
  float lon;
  float alt;
  float temp;
  float pressure;
  float humidity;
  uint8_t sat;
  uint8_t padding[3];
};

static_assert(
  sizeof(TelemetryPacket) == 32,
  "TelemetryPacket must be exactly 32 bytes"
);

TelemetryPacket packet;

void connectWiFi();
void sendToServer(const TelemetryPacket& p);

void setup() {
  Serial.begin(115200);
  delay(1000);

  connectWiFi();

  // SCK = 18, MISO = 19, MOSI = 23, CSN = 26
  SPI.begin(18, 19, 23, NRF_CSN);

  if (!radio.begin()) {
    Serial.println("nRF24 not detected");
    while (true) {
      delay(1000);
    }
  }

  radio.openReadingPipe(0, address);
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.startListening();

  Serial.print("Expected packet size: ");
  Serial.println(sizeof(TelemetryPacket));

  Serial.println("Ground station ready: nRF24 RX + HTTP POST");
}

void loop() {
  if (radio.available()) {
    radio.read(&packet, sizeof(packet));

    Serial.println();
    Serial.println("===== PACKET RECEIVED =====");

    Serial.print("Counter: ");
    Serial.println(packet.counter);

    Serial.print("Latitude: ");
    Serial.println(packet.lat, 6);

    Serial.print("Longitude: ");
    Serial.println(packet.lon, 6);

    Serial.print("Altitude: ");
    Serial.println(packet.alt, 2);

    Serial.print("Satellites: ");
    Serial.println(packet.sat);

    Serial.print("Temperature: ");
    Serial.println(packet.temp, 2);

    Serial.print("Pressure: ");
    Serial.println(packet.pressure, 2);

    Serial.print("Humidity: ");
    Serial.println(packet.humidity, 2);

    Serial.print("Mission mode: ");
    Serial.println(packet.padding[0]);

    Serial.print("GPS status: ");
    Serial.println(packet.padding[1]);

    sendToServer(packet);
  }
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void sendToServer(const TelemetryPacket& p) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");

    connectWiFi();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Could not reconnect to WiFi");
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

  Serial.println("Sending JSON:");
  Serial.println(json);

  int httpCode = http.POST(json);

  Serial.print("HTTP response: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    Serial.println(http.getString());
  } else {
    Serial.print("HTTP error: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}