##include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_now.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// =========================
// WiFi + server
// =========================
const char* ssid = "Inaki ";
const char* password = "bb21cbe07284";
const char* serverUrl = "https://cansat1.onrender.com/api/telemetry";

// =========================
// ESP-NOW
// =========================
// The ESP32-CAM is currently configured on channel 1.
// The ground-station WiFi AP/hotspot must therefore also use channel 1.
const uint8_t ESPNOW_CHANNEL = 1;

// =========================
// Telemetry protocol
// Must match onboard ESP32 and ESP32-CAM exactly
// =========================
enum PacketType : uint8_t {
  PACKET_CORE = 1,
  PACKET_ATTITUDE = 2
};

const uint8_t PACKET_VERSION = 1;

struct __attribute__((packed)) TelemetryPacket {
  uint8_t packetType;
  uint8_t version;

  uint32_t counter;
  uint32_t time_ms;

  float lat;
  float lon;
  float alt;
  float temp;
  float pressure;
  float humidity;

  uint8_t sat;
  uint8_t reserved[1];
};

struct __attribute__((packed)) TelemetryAttitudePacket {
  uint8_t packetType;
  uint8_t version;

  uint32_t counter;
  uint32_t time_ms;

  uint16_t lidar_mm;

  int16_t roll_deg10;
  int16_t pitch_deg10;
  int16_t yaw_deg10;

  uint8_t mode;
  uint8_t lidar_status;
  uint8_t mpu_status;

  uint8_t reserved[11];
};

static_assert(sizeof(TelemetryPacket) == 36, "TelemetryPacket must be 36 bytes");
static_assert(sizeof(TelemetryAttitudePacket) == 32, "TelemetryAttitudePacket must be 32 bytes");

// =========================
// ESP-NOW RX queue
// Keep the callback short: copy packet here, process HTTP in loop().
// =========================
const size_t MAX_ESPNOW_PACKET_SIZE = sizeof(TelemetryPacket);
const uint8_t ESPNOW_QUEUE_LENGTH = 32;

struct ReceivedEspNowPacket {
  uint8_t data[MAX_ESPNOW_PACKET_SIZE];
  uint8_t len;
};

QueueHandle_t espNowQueue = nullptr;
volatile uint32_t droppedEspNowPackets = 0;

// =========================
// Function declarations
// =========================
void connectWiFi();
bool initEspNow();
void processReceivedPacket(const ReceivedEspNowPacket& rx);
void processCorePacket(const TelemetryPacket& p);
void processAttitudePacket(const TelemetryAttitudePacket& p);
void sendToServer(const TelemetryPacket& p);
void checkRadioChannel();

void queueEspNowPacket(const uint8_t* data, int len) {
  if (espNowQueue == nullptr || data == nullptr) {
    return;
  }

  if (len <= 0 || len > (int)MAX_ESPNOW_PACKET_SIZE) {
    return;
  }

  ReceivedEspNowPacket rx = {};
  rx.len = (uint8_t)len;
  memcpy(rx.data, data, len);

  if (xQueueSend(espNowQueue, &rx, 0) != pdTRUE) {
    droppedEspNowPackets++;
  }
}

// Arduino-ESP32 3.x / ESP-IDF 5.x callback signature
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  queueEspNowPacket(data, len);
}
#else
// Compatibility with older Arduino-ESP32 cores
void onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  queueEspNowPacket(data, len);
}
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  connectWiFi();
  checkRadioChannel();

  espNowQueue = xQueueCreate(ESPNOW_QUEUE_LENGTH, sizeof(ReceivedEspNowPacket));
  if (espNowQueue == nullptr) {
    Serial.println("Error creando cola ESP-NOW");
    while (1) {
      delay(1000);
    }
  }

  if (!initEspNow()) {
    Serial.println("No se pudo iniciar ESP-NOW");
    while (1) {
      delay(1000);
    }
  }

  Serial.print("CORE packet size: ");
  Serial.println(sizeof(TelemetryPacket));

  Serial.print("ATTITUDE packet size: ");
  Serial.println(sizeof(TelemetryAttitudePacket));

  Serial.println("Ground lista: ESP-NOW RX + HTTP POST");
}

void loop() {
  ReceivedEspNowPacket rx;

  while (xQueueReceive(espNowQueue, &rx, 0) == pdTRUE) {
    processReceivedPacket(rx);
  }

  static uint32_t lastReportedDrops = 0;
  uint32_t currentDrops = droppedEspNowPackets;

  if (currentDrops != lastReportedDrops) {
    Serial.print("ESP-NOW packets dropped because RX queue was full: ");
    Serial.println(currentDrops);
    lastReportedDrops = currentDrops;
  }

  delay(1);
}

bool initEspNow() {
  // WiFi is already in STA mode because connectWiFi() connected to the AP.
  // ESP-NOW will use that same radio/channel.
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return false;
  }

  if (esp_now_register_recv_cb(onDataReceived) != ESP_OK) {
    Serial.println("Error registrando callback ESP-NOW RX");
    return false;
  }

  Serial.println("ESP-NOW RX iniciado");
  return true;
}

void processReceivedPacket(const ReceivedEspNowPacket& rx) {
  if (rx.len < 2) {
    Serial.println("ESP-NOW packet demasiado corto");
    return;
  }

  uint8_t packetType = rx.data[0];
  uint8_t version = rx.data[1];

  if (version != PACKET_VERSION) {
    Serial.print("Version de paquete no soportada: ");
    Serial.println(version);
    return;
  }

  if (packetType == PACKET_CORE) {
    if (rx.len != sizeof(TelemetryPacket)) {
      Serial.print("CORE con tamaño incorrecto: ");
      Serial.println(rx.len);
      return;
    }

    TelemetryPacket packet;
    memcpy(&packet, rx.data, sizeof(packet));
    processCorePacket(packet);
    return;
  }

  if (packetType == PACKET_ATTITUDE) {
    if (rx.len != sizeof(TelemetryAttitudePacket)) {
      Serial.print("ATTITUDE con tamaño incorrecto: ");
      Serial.println(rx.len);
      return;
    }

    TelemetryAttitudePacket packet;
    memcpy(&packet, rx.data, sizeof(packet));
    processAttitudePacket(packet);
    return;
  }

  Serial.print("Tipo de paquete ESP-NOW desconocido: ");
  Serial.println(packetType);
}

void processCorePacket(const TelemetryPacket& p) {
  Serial.println("\n===== CORE PACKET RECEIVED =====");

  Serial.print("Counter: ");
  Serial.println(p.counter);

  Serial.print("Mission time [ms]: ");
  Serial.println(p.time_ms);

  Serial.print("Lat: ");
  Serial.println(p.lat, 6);

  Serial.print("Lon: ");
  Serial.println(p.lon, 6);

  Serial.print("Alt: ");
  Serial.println(p.alt, 2);

  Serial.print("Sat: ");
  Serial.println(p.sat);

  Serial.print("BME Temp: ");
  Serial.println(p.temp, 2);

  Serial.print("BME Pressure: ");
  Serial.println(p.pressure, 2);

  Serial.print("BME Humidity: ");
  Serial.println(p.humidity, 2);

  sendToServer(p);
}

void processAttitudePacket(const TelemetryAttitudePacket& p) {
  Serial.println("\n===== ATTITUDE PACKET RECEIVED =====");

  Serial.print("Counter: ");
  Serial.println(p.counter);

  Serial.print("Mission time [ms]: ");
  Serial.println(p.time_ms);

  Serial.print("LiDAR [mm]: ");
  Serial.println(p.lidar_mm);

  Serial.print("Roll [deg]: ");
  Serial.println(p.roll_deg10 / 10.0f, 1);

  Serial.print("Pitch [deg]: ");
  Serial.println(p.pitch_deg10 / 10.0f, 1);

  Serial.print("Yaw [deg]: ");
  Serial.println(p.yaw_deg10 / 10.0f, 1);

  Serial.print("Mode: ");
  Serial.println(p.mode);

  Serial.print("LiDAR status: ");
  Serial.println(p.lidar_status);

  Serial.print("MPU status: ");
  Serial.println(p.mpu_status);

  // The current backend only consumes the CORE telemetry fields.
  // ATTITUDE is received correctly here but is not posted yet.
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);

  Serial.print("Conectando a WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Ground STA MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("WiFi channel: ");
  Serial.println(WiFi.channel());
}

void checkRadioChannel() {
  uint8_t currentChannel = WiFi.channel();

  if (currentChannel != ESPNOW_CHANNEL) {
    Serial.println("\n*** WARNING: ESP-NOW CHANNEL MISMATCH ***");
    Serial.print("ESP32-CAM expected channel: ");
    Serial.println(ESPNOW_CHANNEL);
    Serial.print("Ground WiFi channel: ");
    Serial.println(currentChannel);
    Serial.println("Configure the hotspot/router to channel 1 or match the CAM channel.");
  } else {
    Serial.println("ESP-NOW/WiFi channel OK");
  }
}

void sendToServer(const TelemetryPacket& p) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Reintentando...");
    connectWiFi();
    checkRadioChannel();

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
