#include <WiFi.h>

// ============================================================#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_now.h>
#include <esp_idf_version.h>

// ============================================================
// WiFi + backend
// ============================================================

const char* ssid = "LaboPLC";
const char* password = "link1982";

const char* serverUrl =
  "https://cansat1.onrender.com/api/telemetry";


// ============================================================
// ESP-NOW
//
// Must match the ESP32-CAM channel.
// Because this ESP32 also connects to WiFi, the access point
// must use the same channel as the ESP32-CAM.
// ============================================================

#define ESPNOW_CHANNEL 1


// ============================================================
// Unified Hermes telemetry protocol
//
// MUST match the onboard ESP32 and ESP32-CAM exactly.
// ============================================================

struct __attribute__((packed)) TelemetryPacket {
  // Mission
  uint32_t counter;
  uint32_t time_ms;

  // GPS
  float lat;
  float lon;
  float alt;
  float speed;          // km/h
  uint8_t sat;

  // BME680
  float temp;           // deg C
  float humidity;       // %
  float pressure;       // hPa
  float gas_kohm;       // kOhm

  // MPU6050 acceleration
  int16_t accel_x_cms2;       // cm/s^2
  int16_t accel_y_cms2;       // cm/s^2
  int16_t accel_z_cms2;       // cm/s^2
  int16_t accel_total_cms2;   // cm/s^2

  // Mission state
  uint8_t mode;
};

static_assert(
  sizeof(TelemetryPacket) == 50,
  "TelemetryPacket must be exactly 50 bytes"
);


// ============================================================
// ESP-NOW RX queue
//
// The ESP-NOW callback only copies data into the queue.
// HTTPS POST is done later from loop(), never from the callback.
// ============================================================

constexpr size_t MAX_PACKET_SIZE =
  sizeof(TelemetryPacket);

constexpr uint8_t RX_QUEUE_LENGTH = 32;

struct ReceivedFrame {
  uint8_t sourceMac[6];
  int8_t rssi;
  uint8_t length;
  uint8_t data[MAX_PACKET_SIZE];
};

QueueHandle_t rxQueue = nullptr;


// ============================================================
// Statistics
// ============================================================

volatile uint32_t queueDrops = 0;

uint32_t telemetryReceived = 0;
uint32_t invalidReceived = 0;

uint32_t httpOk = 0;
uint32_t httpErrors = 0;

uint32_t missingPackets = 0;
uint32_t duplicatePackets = 0;
uint32_t outOfOrderPackets = 0;

uint32_t lastCounter = 0;
bool hasLastCounter = false;


// ============================================================
// Function declarations
// ============================================================

void connectWiFi();
bool initEspNow();
void checkRadioChannel();

void processReceivedFrame(
  const ReceivedFrame &frame
);

bool sendTelemetryToServer(
  const TelemetryPacket &packet,
  int8_t espNowRssi
);

const char* missionModeName(
  uint8_t mode
);

void printMac(
  const uint8_t *mac
);

void updatePacketLossStats(
  uint32_t counter
);


// ============================================================
// Helpers
// ============================================================

void printMac(
  const uint8_t *mac
) {
  if (mac == nullptr) {
    Serial.print("--:--:--:--:--:--");
    return;
  }

  for (int i = 0; i < 6; i++) {
    if (i > 0) {
      Serial.print(":");
    }

    if (mac[i] < 0x10) {
      Serial.print("0");
    }

    Serial.print(
      mac[i],
      HEX
    );
  }
}


const char* missionModeName(
  uint8_t mode
) {
  switch (mode) {
    case 0:
      return "PRELAUNCH";

    case 1:
      return "DESCENT";

    case 2:
      return "POST_IMPACT";

    default:
      return "UNKNOWN";
  }
}


// ============================================================
// Packet loss / counter statistics
// ============================================================

void updatePacketLossStats(
  uint32_t counter
) {
  if (!hasLastCounter) {
    lastCounter = counter;
    hasLastCounter = true;
    return;
  }

  if (counter == lastCounter) {
    duplicatePackets++;
    return;
  }

  if (counter > lastCounter) {
    uint32_t difference =
      counter - lastCounter;

    if (difference > 1) {
      missingPackets +=
        difference - 1;
    }

    lastCounter = counter;
    return;
  }

  // Counter went backwards.
  // This can happen after a transmitter reboot or due to
  // an out-of-order packet.
  outOfOrderPackets++;
  lastCounter = counter;
}


// ============================================================
// ESP-NOW callback compatibility
// ============================================================

#if ESP_IDF_VERSION_MAJOR > 5 || \
    (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 1)

void onDataReceived(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  if (
    data == nullptr ||
    len != (int)sizeof(TelemetryPacket)
  ) {
    invalidReceived++;
    return;
  }

  ReceivedFrame frame = {};

  frame.length =
    (uint8_t)len;

  memcpy(
    frame.data,
    data,
    len
  );

  if (info != nullptr) {
    if (info->src_addr != nullptr) {
      memcpy(
        frame.sourceMac,
        info->src_addr,
        6
      );
    }

    if (info->rx_ctrl != nullptr) {
      frame.rssi =
        info->rx_ctrl->rssi;
    }
  }

  if (
    xQueueSend(
      rxQueue,
      &frame,
      0
    ) != pdTRUE
  ) {
    queueDrops++;
  }
}

#else

void onDataReceived(
  const uint8_t *mac,
  const uint8_t *data,
  int len
) {
  if (
    data == nullptr ||
    len != (int)sizeof(TelemetryPacket)
  ) {
    invalidReceived++;
    return;
  }

  ReceivedFrame frame = {};

  frame.length =
    (uint8_t)len;

  frame.rssi = 0;

  memcpy(
    frame.data,
    data,
    len
  );

  if (mac != nullptr) {
    memcpy(
      frame.sourceMac,
      mac,
      6
    );
  }

  if (
    xQueueSend(
      rxQueue,
      &frame,
      0
    ) != pdTRUE
  ) {
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
  Serial.println(
    "=========================================="
  );
  Serial.println(
    "Hermes ground station"
  );
  Serial.println(
    "ESP-NOW unified telemetry -> HTTPS backend"
  );
  Serial.println(
    "=========================================="
  );

  rxQueue =
    xQueueCreate(
      RX_QUEUE_LENGTH,
      sizeof(ReceivedFrame)
    );

  if (rxQueue == nullptr) {
    Serial.println(
      "ERROR: could not create ESP-NOW RX queue"
    );

    while (true) {
      delay(1000);
    }
  }

  // WiFi connection fixes the ESP32 radio to the AP channel.
  connectWiFi();

  checkRadioChannel();

  if (!initEspNow()) {
    Serial.println(
      "ERROR: ESP-NOW initialization failed"
    );

    while (true) {
      delay(1000);
    }
  }

  Serial.print("Ground MAC: ");
  Serial.println(
    WiFi.macAddress()
  );

  Serial.print(
    "Telemetry packet size: "
  );
  Serial.print(
    sizeof(TelemetryPacket)
  );
  Serial.println(" bytes");

  Serial.println(
    "Waiting for telemetry..."
  );
}


// ============================================================
// Main loop
// ============================================================

void loop() {
  ReceivedFrame frame;

  while (
    xQueueReceive(
      rxQueue,
      &frame,
      0
    ) == pdTRUE
  ) {
    processReceivedFrame(
      frame
    );
  }

  static uint32_t lastStatsMs = 0;

  if (
    millis() - lastStatsMs >= 5000
  ) {
    lastStatsMs = millis();

    Serial.println();
    Serial.println(
      "---------------- GROUND STATS ----------------"
    );

    Serial.print(
      "Telemetry received:  "
    );
    Serial.println(
      telemetryReceived
    );

    Serial.print(
      "Invalid frames:      "
    );
    Serial.println(
      invalidReceived
    );

    Serial.print(
      "RX queue drops:      "
    );
    Serial.println(
      queueDrops
    );

    Serial.print(
      "Missing packets:     "
    );
    Serial.println(
      missingPackets
    );

    Serial.print(
      "Duplicate packets:   "
    );
    Serial.println(
      duplicatePackets
    );

    Serial.print(
      "Out-of-order:        "
    );
    Serial.println(
      outOfOrderPackets
    );

    Serial.print(
      "HTTP OK:             "
    );
    Serial.println(
      httpOk
    );

    Serial.print(
      "HTTP errors:         "
    );
    Serial.println(
      httpErrors
    );

    Serial.print(
      "WiFi RSSI:           "
    );
    Serial.print(
      WiFi.RSSI()
    );
    Serial.println(" dBm");

    Serial.print(
      "WiFi channel:        "
    );
    Serial.println(
      WiFi.channel()
    );

    Serial.println(
      "----------------------------------------------"
    );
  }

  delay(1);
}


// ============================================================
// WiFi
// ============================================================

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print(
    "Connecting to WiFi: "
  );
  Serial.println(
    ssid
  );

  WiFi.begin(
    ssid,
    password
  );

  uint32_t started =
    millis();

  while (
    WiFi.status() != WL_CONNECTED
  ) {
    delay(500);
    Serial.print(".");

    if (
      millis() - started > 20000
    ) {
      Serial.println();
      Serial.println(
        "Still waiting for WiFi..."
      );

      started =
        millis();
    }
  }

  Serial.println();
  Serial.println(
    "WiFi connected"
  );

  Serial.print(
    "IP address: "
  );
  Serial.println(
    WiFi.localIP()
  );

  Serial.print(
    "WiFi channel: "
  );
  Serial.println(
    WiFi.channel()
  );
}


// ============================================================
// WiFi / ESP-NOW channel check
// ============================================================

void checkRadioChannel() {
  uint8_t currentChannel =
    WiFi.channel();

  if (
    currentChannel != ESPNOW_CHANNEL
  ) {
    Serial.println();
    Serial.println(
      "**************************************************"
    );
    Serial.println(
      "WARNING: ESP-NOW / WiFi CHANNEL MISMATCH"
    );

    Serial.print(
      "ESP32-CAM channel: "
    );
    Serial.println(
      ESPNOW_CHANNEL
    );

    Serial.print(
      "Ground WiFi channel: "
    );
    Serial.println(
      currentChannel
    );

    Serial.println(
      "The access point and ESP32-CAM must use"
    );
    Serial.println(
      "the same WiFi channel for ESP-NOW to work."
    );

    Serial.println(
      "**************************************************"
    );
    Serial.println();
  }

  else {
    Serial.println(
      "ESP-NOW/WiFi channel OK"
    );
  }
}


// ============================================================
// ESP-NOW initialization
// ============================================================

bool initEspNow() {
  if (
    esp_now_init() != ESP_OK
  ) {
    Serial.println(
      "ERROR initializing ESP-NOW"
    );
    return false;
  }

  if (
    esp_now_register_recv_cb(
      onDataReceived
    ) != ESP_OK
  ) {
    Serial.println(
      "ERROR registering ESP-NOW receive callback"
    );
    return false;
  }

  Serial.println(
    "ESP-NOW RX initialized"
  );

  return true;
}


// ============================================================
// Process unified telemetry packet
// ============================================================

void processReceivedFrame(
  const ReceivedFrame &frame
) {
  if (
    frame.length !=
    sizeof(TelemetryPacket)
  ) {
    invalidReceived++;

    Serial.print(
      "Telemetry wrong size: "
    );
    Serial.print(
      frame.length
    );

    Serial.print(
      " expected "
    );
    Serial.println(
      sizeof(TelemetryPacket)
    );

    return;
  }

  TelemetryPacket packet;

  memcpy(
    &packet,
    frame.data,
    sizeof(packet)
  );

  // Simple protocol sanity check.
  if (packet.mode > 2) {
    invalidReceived++;

    Serial.print(
      "Invalid mission mode: "
    );
    Serial.println(
      packet.mode
    );

    return;
  }

  telemetryReceived++;

  updatePacketLossStats(
    packet.counter
  );

  Serial.println();
  Serial.println(
    "===== TELEMETRY RX ====="
  );

  Serial.print(
    "Source MAC: "
  );
  printMac(
    frame.sourceMac
  );
  Serial.println();

  Serial.print(
    "RSSI: "
  );
  Serial.print(
    frame.rssi
  );
  Serial.println(" dBm");

  Serial.print(
    "Counter: "
  );
  Serial.println(
    packet.counter
  );

  Serial.print(
    "Mission time ms: "
  );
  Serial.println(
    packet.time_ms
  );

  Serial.print(
    "GPS: "
  );
  Serial.print(
    packet.lat,
    6
  );
  Serial.print(", ");
  Serial.println(
    packet.lon,
    6
  );

  Serial.print(
    "Altitude m: "
  );
  Serial.println(
    packet.alt,
    2
  );

  Serial.print(
    "Speed km/h: "
  );
  Serial.println(
    packet.speed,
    2
  );

  Serial.print(
    "Satellites: "
  );
  Serial.println(
    packet.sat
  );

  Serial.print(
    "Temperature C: "
  );
  Serial.println(
    packet.temp,
    2
  );

  Serial.print(
    "Humidity %: "
  );
  Serial.println(
    packet.humidity,
    2
  );

  Serial.print(
    "Pressure hPa: "
  );
  Serial.println(
    packet.pressure,
    2
  );

  Serial.print(
    "Gas kOhm: "
  );
  Serial.println(
    packet.gas_kohm,
    2
  );

  Serial.print(
    "Accel X m/s2: "
  );
  Serial.println(
    packet.accel_x_cms2 /
    100.0f,
    2
  );

  Serial.print(
    "Accel Y m/s2: "
  );
  Serial.println(
    packet.accel_y_cms2 /
    100.0f,
    2
  );

  Serial.print(
    "Accel Z m/s2: "
  );
  Serial.println(
    packet.accel_z_cms2 /
    100.0f,
    2
  );

  Serial.print(
    "Accel total m/s2: "
  );
  Serial.println(
    packet.accel_total_cms2 /
    100.0f,
    2
  );

  Serial.print(
    "Mission mode: "
  );
  Serial.print(
    packet.mode
  );
  Serial.print(" (");
  Serial.print(
    missionModeName(
      packet.mode
    )
  );
  Serial.println(")");

  bool ok =
    sendTelemetryToServer(
      packet,
      frame.rssi
    );

  if (ok) {
    httpOk++;
  }

  else {
    httpErrors++;
  }
}


// ============================================================
// HTTPS POST
//
// Binary telemetry uses compact acceleration values in cm/s^2.
// JSON converts them back to m/s^2 for easier backend/dashboard
// interpretation.
// ============================================================

bool sendTelemetryToServer(
  const TelemetryPacket &packet,
  int8_t espNowRssi
) {
  if (
    WiFi.status() != WL_CONNECTED
  ) {
    Serial.println(
      "WiFi disconnected. Reconnecting..."
    );

    connectWiFi();

    checkRadioChannel();

    if (
      WiFi.status() != WL_CONNECTED
    ) {
      Serial.println(
        "Could not reconnect to WiFi"
      );

      return false;
    }
  }

  String json;
  json.reserve(512);

  json = "{";

  json +=
    "\"counter\":" +
    String(packet.counter) +
    ",";

  json +=
    "\"time_ms\":" +
    String(packet.time_ms) +
    ",";

  json +=
    "\"lat\":" +
    String(packet.lat, 6) +
    ",";

  json +=
    "\"lon\":" +
    String(packet.lon, 6) +
    ",";

  json +=
    "\"alt\":" +
    String(packet.alt, 2) +
    ",";

  json +=
    "\"speed\":" +
    String(packet.speed, 2) +
    ",";

  json +=
    "\"sat\":" +
    String(packet.sat) +
    ",";

  json +=
    "\"temp\":" +
    String(packet.temp, 2) +
    ",";

  json +=
    "\"humidity\":" +
    String(packet.humidity, 2) +
    ",";

  json +=
    "\"pressure\":" +
    String(packet.pressure, 2) +
    ",";

  json +=
    "\"gas_kohm\":" +
    String(packet.gas_kohm, 2) +
    ",";

  json +=
    "\"accel_x\":" +
    String(
      packet.accel_x_cms2 /
      100.0f,
      2
    ) +
    ",";

  json +=
    "\"accel_y\":" +
    String(
      packet.accel_y_cms2 /
      100.0f,
      2
    ) +
    ",";

  json +=
    "\"accel_z\":" +
    String(
      packet.accel_z_cms2 /
      100.0f,
      2
    ) +
    ",";

  json +=
    "\"accel_total\":" +
    String(
      packet.accel_total_cms2 /
      100.0f,
      2
    ) +
    ",";

  json +=
    "\"mode\":" +
    String(packet.mode) +
    ",";

  json +=
    "\"espnow_rssi\":" +
    String(espNowRssi);

  json += "}";


  WiFiClientSecure client;

  // Current prototype uses HTTPS without CA validation.
  client.setInsecure();


  HTTPClient http;

  http.setTimeout(5000);


  if (
    !http.begin(
      client,
      serverUrl
    )
  ) {
    Serial.println(
      "HTTP begin failed"
    );

    return false;
  }


  http.addHeader(
    "Content-Type",
    "application/json"
  );


  Serial.print(
    "POST counter="
  );
  Serial.println(
    packet.counter
  );


  int httpCode =
    http.POST(
      json
    );


  Serial.print(
    "HTTP response: "
  );
  Serial.println(
    httpCode
  );


  bool ok =
    (
      httpCode >= 200 &&
      httpCode < 300
    );


  if (httpCode > 0) {
    String response =
      http.getString();

    if (
      response.length() > 0
    ) {
      Serial.print(
        "Server: "
      );
      Serial.println(
        response
      );
    }
  }

  else {
    Serial.print(
      "HTTP error: "
    );
    Serial.println(
      http.errorToString(
        httpCode
      )
    );
  }


  http.end();

  return ok;
}