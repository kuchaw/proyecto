#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_now.h>
#include <esp_idf_version.h>

// ============================================================
// WiFi + backend
// Preserved from the original receiver.ino
// ============================================================
const char* ssid = "Sf2026";
const char* password = "16111505";
const char* serverUrl = "https://cansat1.onrender.com/api/telemetry";

// ============================================================
// ESP-NOW
// Must match the ESP32-CAM transmitter channel.
// IMPORTANT: the WiFi AP used by this ground ESP32 must also
// operate on this channel, because WiFi + ESP-NOW share the
// same 2.4 GHz radio.
// ============================================================
#define ESPNOW_CHANNEL 1

// ============================================================
// Telemetry protocol
// MUST match onboard ESP32 and ESP32-CAM exactly.
// ============================================================
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

static_assert(sizeof(TelemetryPacket) == 36,
              "TelemetryPacket must be exactly 36 bytes");
static_assert(sizeof(TelemetryAttitudePacket) == 32,
              "TelemetryAttitudePacket must be exactly 32 bytes");

// ============================================================
// ESP-NOW RX queue
// The receive callback only copies bytes into this queue.
// Printing and HTTP are performed from loop().
// ============================================================
constexpr size_t MAX_PACKET_SIZE = sizeof(TelemetryPacket);
constexpr uint8_t RX_QUEUE_LENGTH = 32;

struct ReceivedFrame {
  uint8_t sourceMac[6];
  int8_t rssi;
  uint8_t length;
  uint8_t data[MAX_PACKET_SIZE];
};

QueueHandle_t rxQueue = nullptr;

volatile uint32_t queueDrops = 0;
uint32_t coreReceived = 0;
uint32_t attitudeReceived = 0;
uint32_t invalidReceived = 0;
uint32_t httpOk = 0;
uint32_t httpErrors = 0;

// ============================================================
// Function declarations
// ============================================================
void connectWiFi();
bool initEspNow();
void checkRadioChannel();
void processReceivedFrame(const ReceivedFrame &frame);
void processCorePacket(const ReceivedFrame &frame);
void processAttitudePacket(const ReceivedFrame &frame);
bool sendCoreToServer(const TelemetryPacket &p, int8_t rssi);
void printMac(const uint8_t *mac);
const char *missionModeName(uint8_t mode);

// ============================================================
// Helpers
// ============================================================
void printMac(const uint8_t *mac) {
  if (mac == nullptr) {
    Serial.print("--:--:--:--:--:--");
    return;
  }

  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (mac[i] < 0x10) Serial.print("0");
    Serial.print(mac[i], HEX);
  }
}

const char *missionModeName(uint8_t mode) {
  switch (mode) {
    case 0: return "PRELAUNCH";
    case 1: return "DESCENT";
    case 2: return "POST_IMPACT";
    default: return "UNKNOWN";
  }
}

// ============================================================
// ESP-NOW receive callback
// Arduino-ESP32 / ESP-IDF 5.1+ uses esp_now_recv_info_t.
// Older cores use the legacy MAC-pointer signature.
// ============================================================
#if ESP_IDF_VERSION_MAJOR > 5 || \
    (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 1)

void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *data,
                    int len) {
  if (data == nullptr || len <= 0 || len > (int)MAX_PACKET_SIZE) {
    invalidReceived++;
    return;
  }

  ReceivedFrame frame = {};
  frame.length = (uint8_t)len;
  memcpy(frame.data, data, len);

  if (info != nullptr) {
    if (info->src_addr != nullptr) {
      memcpy(frame.sourceMac, info->src_addr, 6);
    }

    if (info->rx_ctrl != nullptr) {
      frame.rssi = info->rx_ctrl->rssi;
    }
  }

  if (xQueueSend(rxQueue, &frame, 0) != pdTRUE) {
    queueDrops++;
  }
}

#else

void onDataReceived(const uint8_t *mac,
                    const uint8_t *data,
                    int len) {
  if (data == nullptr || len <= 0 || len > (int)MAX_PACKET_SIZE) {
    invalidReceived++;
    return;
  }

  ReceivedFrame frame = {};
  frame.length = (uint8_t)len;
  frame.rssi = 0;
  memcpy(frame.data, data, len);

  if (mac != nullptr) {
    memcpy(frame.sourceMac, mac, 6);
  }

  if (xQueueSend(rxQueue, &frame, 0) != pdTRUE) {
    queueDrops++;
  }
}

#endif

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Hermes ground station");
  Serial.println("ESP-NOW RX + HTTPS backend");
  Serial.println("---------------------------------");

  rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(ReceivedFrame));
  if (rxQueue == nullptr) {
    Serial.println("ERROR: could not create ESP-NOW RX queue");
    while (true) delay(1000);
  }

  // Connect first. The ESP32 will then use the WiFi AP channel.
  connectWiFi();
  checkRadioChannel();

  if (!initEspNow()) {
    Serial.println("ERROR: ESP-NOW initialization failed");
    while (true) delay(1000);
  }

  Serial.print("Ground MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("CORE packet size: ");
  Serial.print(sizeof(TelemetryPacket));
  Serial.println(" bytes");

  Serial.print("ATTITUDE packet size: ");
  Serial.print(sizeof(TelemetryAttitudePacket));
  Serial.println(" bytes");

  Serial.println();
  Serial.println("Waiting for telemetry...");
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  ReceivedFrame frame;

  // Process all frames accumulated while an HTTP POST was running.
  while (xQueueReceive(rxQueue, &frame, 0) == pdTRUE) {
    processReceivedFrame(frame);
  }

  static unsigned long lastStatsMs = 0;
  if (millis() - lastStatsMs >= 5000) {
    lastStatsMs = millis();

    Serial.println();
    Serial.println("---------------- GROUND STATS ----------------");
    Serial.print("CORE received:     "); Serial.println(coreReceived);
    Serial.print("ATT received:      "); Serial.println(attitudeReceived);
    Serial.print("Invalid frames:    "); Serial.println(invalidReceived);
    Serial.print("RX queue drops:    "); Serial.println(queueDrops);
    Serial.print("HTTP OK:           "); Serial.println(httpOk);
    Serial.print("HTTP errors:       "); Serial.println(httpErrors);
    Serial.print("WiFi RSSI:         "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    Serial.print("WiFi channel:      "); Serial.println(WiFi.channel());
    Serial.println("----------------------------------------------");
  }

  delay(1);
}

// ============================================================
// WiFi / ESP-NOW initialization
// ============================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  unsigned long started = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    // Keep trying, but print a useful diagnostic periodically.
    if (millis() - started > 20000) {
      Serial.println();
      Serial.println("Still waiting for WiFi...");
      started = millis();
    }
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("WiFi channel: ");
  Serial.println(WiFi.channel());
}

void checkRadioChannel() {
  uint8_t currentChannel = WiFi.channel();

  if (currentChannel != ESPNOW_CHANNEL) {
    Serial.println();
    Serial.println("**************************************************");
    Serial.println("WARNING: ESP-NOW / WiFi CHANNEL MISMATCH");
    Serial.print("ESP32-CAM channel: ");
    Serial.println(ESPNOW_CHANNEL);
    Serial.print("Ground WiFi channel: ");
    Serial.println(currentChannel);
    Serial.println("ESP-NOW telemetry will NOT be reliable/received");
    Serial.println("unless both radios use the same channel.");
    Serial.println("**************************************************");
    Serial.println();
  } else {
    Serial.println("ESP-NOW/WiFi channel OK");
  }
}

bool initEspNow() {
  // WiFi is already connected in STA mode. ESP-NOW shares that radio.
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR initializing ESP-NOW");
    return false;
  }

  if (esp_now_register_recv_cb(onDataReceived) != ESP_OK) {
    Serial.println("ERROR registering ESP-NOW receive callback");
    return false;
  }

  Serial.println("ESP-NOW RX initialized");
  return true;
}

// ============================================================
// Packet processing
// ============================================================
void processReceivedFrame(const ReceivedFrame &frame) {
  if (frame.length < 2) {
    invalidReceived++;
    Serial.println("Invalid ESP-NOW frame: too short");
    return;
  }

  uint8_t packetType = frame.data[0];
  uint8_t version = frame.data[1];

  if (version != PACKET_VERSION) {
    invalidReceived++;
    Serial.print("Unsupported packet version: ");
    Serial.println(version);
    return;
  }

  switch (packetType) {
    case PACKET_CORE:
      processCorePacket(frame);
      break;

    case PACKET_ATTITUDE:
      processAttitudePacket(frame);
      break;

    default:
      invalidReceived++;
      Serial.print("Unknown packet type: ");
      Serial.println(packetType);
      break;
  }
}

void processCorePacket(const ReceivedFrame &frame) {
  if (frame.length != sizeof(TelemetryPacket)) {
    invalidReceived++;
    Serial.print("CORE wrong size: ");
    Serial.println(frame.length);
    return;
  }

  TelemetryPacket p;
  memcpy(&p, frame.data, sizeof(p));
  coreReceived++;

  Serial.println();
  Serial.println("===== CORE PACKET =====");
  Serial.print("From:        "); printMac(frame.sourceMac); Serial.println();
  Serial.print("ESP-NOW RSSI:"); Serial.print(frame.rssi); Serial.println(" dBm");
  Serial.print("Counter:     "); Serial.println(p.counter);
  Serial.print("Time ms:     "); Serial.println(p.time_ms);
  Serial.print("Lat:         "); Serial.println(p.lat, 6);
  Serial.print("Lon:         "); Serial.println(p.lon, 6);
  Serial.print("Alt:         "); Serial.println(p.alt, 2);
  Serial.print("Sat:         "); Serial.println(p.sat);
  Serial.print("Temp:        "); Serial.println(p.temp, 2);
  Serial.print("Pressure:    "); Serial.println(p.pressure, 2);
  Serial.print("Humidity:    "); Serial.println(p.humidity, 2);
  Serial.print("GPS status:  "); Serial.println(p.reserved[0]);

  // Current server.js consumes the CORE fields.
  if (sendCoreToServer(p, frame.rssi)) {
    httpOk++;
  } else {
    httpErrors++;
  }
}

void processAttitudePacket(const ReceivedFrame &frame) {
  if (frame.length != sizeof(TelemetryAttitudePacket)) {
    invalidReceived++;
    Serial.print("ATTITUDE wrong size: ");
    Serial.println(frame.length);
    return;
  }

  TelemetryAttitudePacket p;
  memcpy(&p, frame.data, sizeof(p));
  attitudeReceived++;

  Serial.println();
  Serial.println("===== ATTITUDE PACKET =====");
  Serial.print("Counter:      "); Serial.println(p.counter);
  Serial.print("Time ms:      "); Serial.println(p.time_ms);
  Serial.print("LiDAR:        "); Serial.print(p.lidar_mm); Serial.println(" mm");
  Serial.print("Roll:         "); Serial.print(p.roll_deg10 / 10.0f, 1); Serial.println(" deg");
  Serial.print("Pitch:        "); Serial.print(p.pitch_deg10 / 10.0f, 1); Serial.println(" deg");
  Serial.print("Yaw:          "); Serial.print(p.yaw_deg10 / 10.0f, 1); Serial.println(" deg");
  Serial.print("Mission mode: "); Serial.print(p.mode);
  Serial.print(" ("); Serial.print(missionModeName(p.mode)); Serial.println(")");
  Serial.print("LiDAR status: "); Serial.println(p.lidar_status);
  Serial.print("MPU status:   "); Serial.println(p.mpu_status);

  // Not posted yet. The current backend does not store these fields.
  // We will connect this packet when server.js is upgraded.
}

// ============================================================
// HTTPS POST
// ============================================================
bool sendCoreToServer(const TelemetryPacket &p, int8_t espNowRssi) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();
    checkRadioChannel();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Could not reconnect to WiFi");
      return false;
    }
  }

  // Keep the fields already understood by server.js.
  // Extra fields are included for diagnostics/future backend upgrades;
  // the current server simply ignores fields it does not store.
  String json;
  json.reserve(256);
  json = "{";
  json += "\"lat\":" + String(p.lat, 6) + ",";
  json += "\"lon\":" + String(p.lon, 6) + ",";
  json += "\"alt\":" + String(p.alt, 2) + ",";
  json += "\"sat\":" + String(p.sat) + ",";
  json += "\"temp\":" + String(p.temp, 2) + ",";
  json += "\"pressure\":" + String(p.pressure, 2) + ",";
  json += "\"humidity\":" + String(p.humidity, 2) + ",";

  // These are not used by the current server yet, but preserving them
  // in the POST makes the ground protocol ready for the next step.
  json += "\"counter\":" + String(p.counter) + ",";
  json += "\"time_ms\":" + String(p.time_ms) + ",";
  json += "\"gps_status\":" + String(p.reserved[0]) + ",";
  json += "\"espnow_rssi\":" + String(espNowRssi);
  json += "}";

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setTimeout(5000);

  if (!http.begin(client, serverUrl)) {
    Serial.println("HTTP begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");

  Serial.println("POST /api/telemetry");
  Serial.println(json);

  int httpCode = http.POST(json);

  Serial.print("HTTP response: ");
  Serial.println(httpCode);

  bool ok = (httpCode >= 200 && httpCode < 300);

  if (httpCode > 0) {
    String response = http.getString();
    if (response.length() > 0) {
      Serial.print("Server: ");
      Serial.println(response);
    }
  } else {
    Serial.print("HTTP error: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
  return ok;
}