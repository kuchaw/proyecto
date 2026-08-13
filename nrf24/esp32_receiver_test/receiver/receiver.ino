#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

// ============================================================
// ESP-NOW test receiver for Hermes telemetry
// Must match the ESP32-CAM transmitter channel.
// ============================================================
#define ESPNOW_CHANNEL 1

// ============================================================
// Packet protocol - MUST match onboard ESP32 and ESP32-CAM
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
// RX queue
// Keep ESP-NOW callback short. Printing is done from loop().
// ============================================================
constexpr size_t MAX_PACKET_SIZE = sizeof(TelemetryPacket);
constexpr uint8_t RX_QUEUE_LENGTH = 20;

struct ReceivedFrame {
  uint8_t sourceMac[6];
  int8_t rssi;
  uint8_t length;
  uint8_t data[MAX_PACKET_SIZE];
};

QueueHandle_t rxQueue = nullptr;

// ============================================================
// Link statistics
// CORE and ATTITUDE have their own loss tracking because they
// share a counter but are transmitted as separate frames.
// ============================================================
uint32_t coreReceived = 0;
uint32_t attitudeReceived = 0;
uint32_t invalidReceived = 0;
uint32_t queueDrops = 0;
uint32_t coreMissing = 0;
uint32_t attitudeMissing = 0;

bool haveLastCoreCounter = false;
bool haveLastAttitudeCounter = false;
uint32_t lastCoreCounter = 0;
uint32_t lastAttitudeCounter = 0;

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

void updateLossCounter(uint32_t currentCounter,
                       bool &haveLast,
                       uint32_t &lastCounter,
                       uint32_t &missingCounter) {
  if (haveLast && currentCounter > lastCounter + 1) {
    missingCounter += currentCounter - lastCounter - 1;
  }

  lastCounter = currentCounter;
  haveLast = true;
}

const char *missionModeName(uint8_t mode) {
  switch (mode) {
    case 0: return "PRELAUNCH";
    case 1: return "DESCENT";
    case 2: return "POST_IMPACT";
    default: return "UNKNOWN";
  }
}

void printFrameHeader(const ReceivedFrame &frame, const char *name) {
  Serial.println();
  Serial.println("==================================================");
  Serial.print(name);
  Serial.print(" | bytes=");
  Serial.print(frame.length);
  Serial.print(" | RSSI=");
  Serial.print(frame.rssi);
  Serial.print(" dBm | from ");
  printMac(frame.sourceMac);
  Serial.println();
}

void processCorePacket(const ReceivedFrame &frame) {
  if (frame.length != sizeof(TelemetryPacket)) {
    Serial.print("CORE rejected: expected ");
    Serial.print(sizeof(TelemetryPacket));
    Serial.print(" bytes, got ");
    Serial.println(frame.length);
    invalidReceived++;
    return;
  }

  TelemetryPacket packet;
  memcpy(&packet, frame.data, sizeof(packet));

  if (packet.version != PACKET_VERSION) {
    Serial.print("CORE warning: packet version ");
    Serial.print(packet.version);
    Serial.print(" != expected ");
    Serial.println(PACKET_VERSION);
  }

  coreReceived++;
  updateLossCounter(packet.counter,
                    haveLastCoreCounter,
                    lastCoreCounter,
                    coreMissing);

  printFrameHeader(frame, "CORE PACKET");

  Serial.print("Counter:      "); Serial.println(packet.counter);
  Serial.print("Time ms:      "); Serial.println(packet.time_ms);
  Serial.print("Latitude:     "); Serial.println(packet.lat, 6);
  Serial.print("Longitude:    "); Serial.println(packet.lon, 6);
  Serial.print("Altitude:     "); Serial.print(packet.alt, 2); Serial.println(" m");
  Serial.print("Satellites:   "); Serial.println(packet.sat);
  Serial.print("Temperature:  "); Serial.print(packet.temp, 2); Serial.println(" C");
  Serial.print("Pressure:     "); Serial.print(packet.pressure, 2); Serial.println(" hPa");
  Serial.print("Humidity:     "); Serial.print(packet.humidity, 2); Serial.println(" %");
  Serial.print("GPS status:   "); Serial.println(packet.reserved[0]);

  Serial.print("CORE received: "); Serial.print(coreReceived);
  Serial.print(" | missing: "); Serial.println(coreMissing);
}

void processAttitudePacket(const ReceivedFrame &frame) {
  if (frame.length != sizeof(TelemetryAttitudePacket)) {
    Serial.print("ATTITUDE rejected: expected ");
    Serial.print(sizeof(TelemetryAttitudePacket));
    Serial.print(" bytes, got ");
    Serial.println(frame.length);
    invalidReceived++;
    return;
  }

  TelemetryAttitudePacket packet;
  memcpy(&packet, frame.data, sizeof(packet));

  if (packet.version != PACKET_VERSION) {
    Serial.print("ATTITUDE warning: packet version ");
    Serial.print(packet.version);
    Serial.print(" != expected ");
    Serial.println(PACKET_VERSION);
  }

  attitudeReceived++;
  updateLossCounter(packet.counter,
                    haveLastAttitudeCounter,
                    lastAttitudeCounter,
                    attitudeMissing);

  printFrameHeader(frame, "ATTITUDE PACKET");

  Serial.print("Counter:      "); Serial.println(packet.counter);
  Serial.print("Time ms:      "); Serial.println(packet.time_ms);
  Serial.print("LiDAR:        "); Serial.print(packet.lidar_mm); Serial.println(" mm");
  Serial.print("Roll:         "); Serial.print(packet.roll_deg10 / 10.0f, 1); Serial.println(" deg");
  Serial.print("Pitch:        "); Serial.print(packet.pitch_deg10 / 10.0f, 1); Serial.println(" deg");
  Serial.print("Yaw:          "); Serial.print(packet.yaw_deg10 / 10.0f, 1); Serial.println(" deg");
  Serial.print("Mission mode: "); Serial.print(packet.mode);
  Serial.print(" ("); Serial.print(missionModeName(packet.mode)); Serial.println(")");
  Serial.print("LiDAR status: "); Serial.println(packet.lidar_status);
  Serial.print("MPU status:   "); Serial.println(packet.mpu_status);

  Serial.print("ATT received:  "); Serial.print(attitudeReceived);
  Serial.print(" | missing: "); Serial.println(attitudeMissing);
}

void processReceivedFrame(const ReceivedFrame &frame) {
  if (frame.length < 2) {
    Serial.println("Invalid ESP-NOW frame: too short");
    invalidReceived++;
    return;
  }

  uint8_t packetType = frame.data[0];

  switch (packetType) {
    case PACKET_CORE:
      processCorePacket(frame);
      break;

    case PACKET_ATTITUDE:
      processAttitudePacket(frame);
      break;

    default:
      Serial.println();
      Serial.print("Unknown packet type: ");
      Serial.print(packetType);
      Serial.print(" | bytes: ");
      Serial.println(frame.length);
      invalidReceived++;
      break;
  }
}

// ============================================================
// ESP-NOW receive callback
// ESP-IDF 5.1+ changed the callback signature.
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
  frame.rssi = 0;  // RSSI is not directly available in the old callback.
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
  Serial.println("Hermes ESP-NOW telemetry receiver");
  Serial.println("---------------------------------");

  rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(ReceivedFrame));
  if (rxQueue == nullptr) {
    Serial.println("ERROR: could not create RX queue");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  esp_err_t channelResult =
      esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (channelResult != ESP_OK) {
    Serial.print("ERROR setting WiFi channel: ");
    Serial.println(channelResult);
    while (true) delay(1000);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR initializing ESP-NOW");
    while (true) delay(1000);
  }

  if (esp_now_register_recv_cb(onDataReceived) != ESP_OK) {
    Serial.println("ERROR registering ESP-NOW receive callback");
    while (true) delay(1000);
  }

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("ESP-NOW channel: ");
  Serial.println(ESPNOW_CHANNEL);

  Serial.print("CORE packet size: ");
  Serial.print(sizeof(TelemetryPacket));
  Serial.println(" bytes");

  Serial.print("ATTITUDE packet size: ");
  Serial.print(sizeof(TelemetryAttitudePacket));
  Serial.println(" bytes");

  Serial.println();
  Serial.println("Waiting for telemetry from ESP32-CAM...");
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  ReceivedFrame frame;

  while (xQueueReceive(rxQueue, &frame, 0) == pdTRUE) {
    processReceivedFrame(frame);
  }

  static unsigned long lastStatsMs = 0;
  if (millis() - lastStatsMs >= 5000) {
    lastStatsMs = millis();

    Serial.println();
    Serial.println("---------------- LINK STATS ----------------");
    Serial.print("CORE received:     "); Serial.println(coreReceived);
    Serial.print("CORE missing:      "); Serial.println(coreMissing);
    Serial.print("ATT received:      "); Serial.println(attitudeReceived);
    Serial.print("ATT missing:       "); Serial.println(attitudeMissing);
    Serial.print("Invalid frames:    "); Serial.println(invalidReceived);
    Serial.print("RX queue drops:    "); Serial.println(queueDrops);
    Serial.println("--------------------------------------------");
  }

  delay(1);
}