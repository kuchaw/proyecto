#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>
#include <limits.h>

// =====================================================
// WiFi + SERVER
// =====================================================
const char* ssid = "Inaki ";
const char* password = "bb21cbe07284";
const char* serverUrl = "https://cansat1.onrender.com/api/telemetry";

// =====================================================
// nRF24 CONFIG
// =====================================================
#define NRF_CE 25
#define NRF_CSN 26

RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "GAAY1";

// =====================================================
// PACKET TYPES - must match flight controller
// =====================================================
enum PacketType : uint8_t {
  PACKET_CORE = 1,
  PACKET_IMU  = 2
};

enum MissionMode : uint8_t {
  MODE_PRE_LAUNCH  = 0,
  MODE_DESCENT     = 1,
  MODE_POST_IMPACT = 2
};

enum StatusFlags : uint8_t {
  FLAG_GPS_VALID     = 1 << 0,
  FLAG_BME_VALID     = 1 << 1,
  FLAG_MPU_VALID     = 1 << 2,
  FLAG_BARO_VALID    = 1 << 3,
  FLAG_BATTERY_VALID = 1 << 4
};

// =====================================================
// SHARED PACKET HEADER
// Must match flight controller exactly
// =====================================================
struct __attribute__((packed)) PacketHeader {
  uint16_t counter;
  uint8_t type;
  uint8_t version;
  uint32_t time_ms;
};

// =====================================================
// CORE PACKET - 32 bytes
// Must match flight controller exactly
// =====================================================
struct __attribute__((packed)) TelemetryCorePacket {
  PacketHeader header;

  int32_t lat_e7;
  int32_t lon_e7;

  int16_t gps_alt_dm;
  int16_t baro_alt_dm;

  int16_t speed_cms;
  int16_t temp_centi;

  uint16_t pressure_x10;
  uint16_t humidity_centi;

  uint8_t sat;
  uint8_t mode;

  uint16_t checksum;
};

static_assert(sizeof(TelemetryCorePacket) == 32, "TelemetryCorePacket must be 32 bytes");

// =====================================================
// IMU PACKET - 32 bytes
// Must match flight controller exactly
// =====================================================
struct __attribute__((packed)) TelemetryImuPacket {
  PacketHeader header;

  int16_t ax_mg;
  int16_t ay_mg;
  int16_t az_mg;

  int16_t gx_dps10;
  int16_t gy_dps10;
  int16_t gz_dps10;

  uint16_t accel_total_mg;
  uint16_t battery_mv;
  uint16_t gas_x10;
  int16_t course_deg10;

  uint8_t flags;
  uint8_t mode;

  uint16_t checksum;
};

static_assert(sizeof(TelemetryImuPacket) == 32, "TelemetryImuPacket must be 32 bytes");

// =====================================================
// MERGED SAMPLE
// This is what gets sent to the server as JSON
// =====================================================
struct TelemetrySample {
  bool hasCore = false;
  bool hasImu = false;
  bool posted = false;

  uint16_t counter = 0;
  uint32_t time_ms = 0;
  uint8_t mode = MODE_PRE_LAUNCH;
  uint8_t flags = 0;

  float lat = NAN;
  float lon = NAN;
  float gps_alt_m = NAN;
  float baro_alt_m = NAN;
  float speed_mps = NAN;
  float speed_kmh = NAN;
  float temp_c = NAN;
  float pressure_hpa = NAN;
  float humidity_pct = NAN;
  uint8_t sat = 0;

  float ax_g = NAN;
  float ay_g = NAN;
  float az_g = NAN;
  float gx_dps = NAN;
  float gy_dps = NAN;
  float gz_dps = NAN;
  float accel_total_g = NAN;
  float battery_v = NAN;
  float gas_kohms = NAN;
  float course_deg = NAN;

  unsigned long lastUpdateLocalMs = 0;
};

TelemetrySample currentSample;

// If CORE arrives but IMU is lost, send CORE-only after this timeout.
const unsigned long MERGE_TIMEOUT_MS = 200;

// =====================================================
// FUNCTION DECLARATIONS
// =====================================================
void connectWiFi();
void processRadio();
void handleRawFrame(uint8_t* buffer);
void handleCorePacket(const TelemetryCorePacket& core);
void handleImuPacket(const TelemetryImuPacket& imu);
void prepareSampleForCounter(uint16_t counter);
void checkPendingSample();
void sendToServer(const TelemetrySample& s);

uint16_t checksum16(const void* data, size_t len);
bool verifyChecksum(const void* data, size_t len, uint16_t receivedChecksum);

float decodeInt16(int16_t value, float scale);
float decodeUInt16(uint16_t value, float scale);
String jsonFloatOrNull(float value, uint8_t digits);
String modeToString(uint8_t mode);

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===== CanSat Ground Station Boot =====");

  connectWiFi();

  SPI.begin(18, 19, 23, NRF_CSN);

  if (!radio.begin()) {
    Serial.println("ERROR: nRF24 not detected");
    while (1);
  }

  radio.openReadingPipe(0, address);
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(32);
  radio.setAutoAck(true);
  radio.startListening();

  Serial.print("CORE packet size: ");
  Serial.println(sizeof(TelemetryCorePacket));

  Serial.print("IMU packet size: ");
  Serial.println(sizeof(TelemetryImuPacket));

  Serial.println("Ground ready: nRF24 RX + packet merge + HTTP POST");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  processRadio();
  checkPendingSample();
}

// =====================================================
// RADIO RECEIVE
// =====================================================
void processRadio() {
  while (radio.available()) {
    uint8_t buffer[32];
    radio.read(&buffer, sizeof(buffer));

    handleRawFrame(buffer);
  }
}

void handleRawFrame(uint8_t* buffer) {
  PacketHeader* header = (PacketHeader*)buffer;

  if (header->type == PACKET_CORE) {
    TelemetryCorePacket core;
    memcpy(&core, buffer, sizeof(core));

    if (!verifyChecksum(&core, sizeof(core), core.checksum)) {
      Serial.println("WARNING: CORE checksum failed");
      return;
    }

    handleCorePacket(core);
  }

  else if (header->type == PACKET_IMU) {
    TelemetryImuPacket imu;
    memcpy(&imu, buffer, sizeof(imu));

    if (!verifyChecksum(&imu, sizeof(imu), imu.checksum)) {
      Serial.println("WARNING: IMU checksum failed");
      return;
    }

    handleImuPacket(imu);
  }

  else {
    Serial.print("WARNING: Unknown packet type: ");
    Serial.println(header->type);
  }
}

// =====================================================
// CORE PACKET HANDLING
// =====================================================
void handleCorePacket(const TelemetryCorePacket& core) {
  prepareSampleForCounter(core.header.counter);

  currentSample.hasCore = true;
  currentSample.counter = core.header.counter;
  currentSample.time_ms = core.header.time_ms;
  currentSample.mode = core.mode;
  currentSample.lastUpdateLocalMs = millis();

  currentSample.lat = core.lat_e7 / 10000000.0f;
  currentSample.lon = core.lon_e7 / 10000000.0f;

  currentSample.gps_alt_m = decodeInt16(core.gps_alt_dm, 10.0f);
  currentSample.baro_alt_m = decodeInt16(core.baro_alt_dm, 10.0f);

  currentSample.speed_mps = decodeInt16(core.speed_cms, 100.0f);
  currentSample.speed_kmh = currentSample.speed_mps * 3.6f;

  currentSample.temp_c = decodeInt16(core.temp_centi, 100.0f);
  currentSample.pressure_hpa = decodeUInt16(core.pressure_x10, 10.0f);
  currentSample.humidity_pct = decodeUInt16(core.humidity_centi, 100.0f);

  currentSample.sat = core.sat;

  Serial.println();
  Serial.println("===== CORE PACKET RECEIVED =====");
  Serial.print("Counter: ");
  Serial.println(currentSample.counter);
  Serial.print("Time ms: ");
  Serial.println(currentSample.time_ms);
  Serial.print("Mode: ");
  Serial.println(modeToString(currentSample.mode));
  Serial.print("Lat: ");
  Serial.println(currentSample.lat, 7);
  Serial.print("Lon: ");
  Serial.println(currentSample.lon, 7);
  Serial.print("GPS Alt m: ");
  Serial.println(currentSample.gps_alt_m, 2);
  Serial.print("Baro Alt m: ");
  Serial.println(currentSample.baro_alt_m, 2);
  Serial.print("Speed km/h: ");
  Serial.println(currentSample.speed_kmh, 2);
  Serial.print("Temp C: ");
  Serial.println(currentSample.temp_c, 2);
  Serial.print("Pressure hPa: ");
  Serial.println(currentSample.pressure_hpa, 2);
  Serial.print("Humidity %: ");
  Serial.println(currentSample.humidity_pct, 2);
  Serial.print("Sat: ");
  Serial.println(currentSample.sat);
}

// =====================================================
// IMU PACKET HANDLING
// =====================================================
void handleImuPacket(const TelemetryImuPacket& imu) {
  prepareSampleForCounter(imu.header.counter);

  currentSample.hasImu = true;
  currentSample.counter = imu.header.counter;
  currentSample.time_ms = imu.header.time_ms;
  currentSample.mode = imu.mode;
  currentSample.flags = imu.flags;
  currentSample.lastUpdateLocalMs = millis();

  currentSample.ax_g = decodeInt16(imu.ax_mg, 1000.0f);
  currentSample.ay_g = decodeInt16(imu.ay_mg, 1000.0f);
  currentSample.az_g = decodeInt16(imu.az_mg, 1000.0f);

  currentSample.gx_dps = decodeInt16(imu.gx_dps10, 10.0f);
  currentSample.gy_dps = decodeInt16(imu.gy_dps10, 10.0f);
  currentSample.gz_dps = decodeInt16(imu.gz_dps10, 10.0f);

  currentSample.accel_total_g = decodeUInt16(imu.accel_total_mg, 1000.0f);
  currentSample.battery_v = decodeUInt16(imu.battery_mv, 1000.0f);
  currentSample.gas_kohms = decodeUInt16(imu.gas_x10, 10.0f);
  currentSample.course_deg = decodeInt16(imu.course_deg10, 10.0f);

  Serial.println();
  Serial.println("===== IMU PACKET RECEIVED =====");
  Serial.print("Counter: ");
  Serial.println(currentSample.counter);
  Serial.print("Mode: ");
  Serial.println(modeToString(currentSample.mode));
  Serial.print("Flags: ");
  Serial.println(currentSample.flags, BIN);
  Serial.print("Accel g: ");
  Serial.print(currentSample.ax_g, 3);
  Serial.print(", ");
  Serial.print(currentSample.ay_g, 3);
  Serial.print(", ");
  Serial.println(currentSample.az_g, 3);
  Serial.print("Gyro dps: ");
  Serial.print(currentSample.gx_dps, 2);
  Serial.print(", ");
  Serial.print(currentSample.gy_dps, 2);
  Serial.print(", ");
  Serial.println(currentSample.gz_dps, 2);
  Serial.print("Accel total g: ");
  Serial.println(currentSample.accel_total_g, 3);
  Serial.print("Battery V: ");
  Serial.println(currentSample.battery_v, 3);
  Serial.print("Gas kOhms: ");
  Serial.println(currentSample.gas_kohms, 2);
  Serial.print("Course deg: ");
  Serial.println(currentSample.course_deg, 1);
}

// =====================================================
// SAMPLE MERGE LOGIC
// =====================================================
void prepareSampleForCounter(uint16_t counter) {
  if (!currentSample.hasCore && !currentSample.hasImu) {
    currentSample.counter = counter;
    return;
  }

  if (currentSample.counter == counter) {
    return;
  }

  // New counter arrived before old sample was posted.
  // If old sample had CORE data, post it even if IMU was lost.
  if (currentSample.hasCore && !currentSample.posted) {
    Serial.println("Posting previous CORE-only sample before switching counter");
    sendToServer(currentSample);
  }

  currentSample = TelemetrySample();
  currentSample.counter = counter;
}

void checkPendingSample() {
  if (currentSample.posted) return;
  if (!currentSample.hasCore) return;

  bool completeSample = currentSample.hasCore && currentSample.hasImu;
  bool timeoutExpired = millis() - currentSample.lastUpdateLocalMs >= MERGE_TIMEOUT_MS;

  if (completeSample || timeoutExpired) {
    sendToServer(currentSample);
  }
}

// =====================================================
// WIFI
// =====================================================
void connectWiFi() {
  Serial.print("Connecting to WiFi");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long startMs = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - startMs > 20000) {
      Serial.println();
      Serial.println("WARNING: WiFi connection timeout");
      return;
    }
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// =====================================================
// HTTP POST
// =====================================================
void sendToServer(const TelemetrySample& s) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("ERROR: Could not reconnect WiFi. Sample not posted.");
      return;
    }
  }

  String json = "{";

  json += "\"counter\":" + String(s.counter) + ",";
  json += "\"time_ms\":" + String(s.time_ms) + ",";
  json += "\"mode\":\"" + modeToString(s.mode) + "\",";
  json += "\"has_core\":" + String(s.hasCore ? "true" : "false") + ",";
  json += "\"has_imu\":" + String(s.hasImu ? "true" : "false") + ",";

  // Fields compatible with current dashboard/server
  json += "\"lat\":" + jsonFloatOrNull(s.lat, 7) + ",";
  json += "\"lon\":" + jsonFloatOrNull(s.lon, 7) + ",";
  json += "\"alt\":" + jsonFloatOrNull(s.gps_alt_m, 2) + ",";
  json += "\"sat\":" + String(s.sat) + ",";
  json += "\"speed\":" + jsonFloatOrNull(s.speed_kmh, 2) + ",";
  json += "\"course\":" + jsonFloatOrNull(s.course_deg, 1) + ",";
  json += "\"temp\":" + jsonFloatOrNull(s.temp_c, 2) + ",";
  json += "\"pressure\":" + jsonFloatOrNull(s.pressure_hpa, 2) + ",";
  json += "\"humidity\":" + jsonFloatOrNull(s.humidity_pct, 2) + ",";
  json += "\"gas\":" + jsonFloatOrNull(s.gas_kohms, 2) + ",";

  // New extended fields
  json += "\"gps_alt\":" + jsonFloatOrNull(s.gps_alt_m, 2) + ",";
  json += "\"baro_alt\":" + jsonFloatOrNull(s.baro_alt_m, 2) + ",";
  json += "\"speed_mps\":" + jsonFloatOrNull(s.speed_mps, 2) + ",";

  json += "\"ax\":" + jsonFloatOrNull(s.ax_g, 3) + ",";
  json += "\"ay\":" + jsonFloatOrNull(s.ay_g, 3) + ",";
  json += "\"az\":" + jsonFloatOrNull(s.az_g, 3) + ",";
  json += "\"gx\":" + jsonFloatOrNull(s.gx_dps, 2) + ",";
  json += "\"gy\":" + jsonFloatOrNull(s.gy_dps, 2) + ",";
  json += "\"gz\":" + jsonFloatOrNull(s.gz_dps, 2) + ",";
  json += "\"accel_total\":" + jsonFloatOrNull(s.accel_total_g, 3) + ",";

  json += "\"battery_v\":" + jsonFloatOrNull(s.battery_v, 3) + ",";
  json += "\"flags\":" + String(s.flags);

  json += "}";

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;

  if (!http.begin(client, serverUrl)) {
    Serial.println("ERROR: HTTP begin failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");

  Serial.println();
  Serial.println("Sending JSON:");
  Serial.println(json);

  int httpCode = http.POST(json);

  Serial.print("HTTP Response: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    Serial.println(http.getString());
  } else {
    Serial.print("HTTP Error: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();

  currentSample.posted = true;
}

// =====================================================
// CHECKSUM
// Must match flight controller checksum function
// =====================================================
uint16_t checksum16(const void* data, size_t len) {
  const uint8_t* bytes = (const uint8_t*)data;
  uint16_t sum = 0;

  // Exclude last two bytes, where checksum is stored.
  for (size_t i = 0; i < len - 2; i++) {
    sum += bytes[i];
  }

  return sum;
}

bool verifyChecksum(const void* data, size_t len, uint16_t receivedChecksum) {
  return checksum16(data, len) == receivedChecksum;
}

// =====================================================
// DECODING HELPERS
// =====================================================
float decodeInt16(int16_t value, float scale) {
  if (value == INT16_MIN) return NAN;
  return value / scale;
}

float decodeUInt16(uint16_t value, float scale) {
  if (value == 0) return NAN;
  return value / scale;
}

String jsonFloatOrNull(float value, uint8_t digits) {
  if (!isfinite(value)) return "null";
  return String(value, digits);
}

String modeToString(uint8_t mode) {
  switch (mode) {
    case MODE_PRE_LAUNCH:
      return "PRE_LAUNCH";

    case MODE_DESCENT:
      return "DESCENT";

    case MODE_POST_IMPACT:
      return "POST_IMPACT";

    default:
      return "UNKNOWN";
  }
}