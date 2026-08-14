#include "esp_camera.h"
#include "SD_MMC.h"
#include "FS.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_M5STACK_ESP32CAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_TTGO_T_JOURNAL

#include "camera_pins.h"


// ======================================================
// File paths
// ======================================================

const char *photoPrefix = "/photos/photo_";
const char *missionLogPath = "/logs/mission.csv";
const char *eventLogPath = "/logs/events.txt";
const char *telemetryLogPath = "/logs/telemetry.csv";


// ======================================================
// UART framing ESP32 -> ESP32-CAM
//
// Frame:
// 0xAA 0x55 | payload_length | payload | XOR checksum
//
// Payload:
// one complete 50-byte TelemetryPacket
// ======================================================

const uint8_t UART_SYNC_1 = 0xAA;
const uint8_t UART_SYNC_2 = 0x55;
const size_t UART_MAX_PAYLOAD = 64;

enum UartRxState : uint8_t {
  UART_WAIT_SYNC_1,
  UART_WAIT_SYNC_2,
  UART_WAIT_LENGTH,
  UART_READ_PAYLOAD,
  UART_WAIT_CHECKSUM
};

UartRxState uartRxState = UART_WAIT_SYNC_1;

uint8_t uartRxBuffer[UART_MAX_PAYLOAD];
uint8_t uartRxLength = 0;
uint8_t uartRxIndex = 0;
uint8_t uartRxChecksum = 0;

uint32_t uartFramesOk = 0;
uint32_t uartChecksumErrors = 0;
uint32_t uartRejectedFrames = 0;


// ======================================================
// Unified Hermes telemetry packet
// Must be identical to the onboard ESP32 definition.
// ======================================================

struct __attribute__((packed)) TelemetryPacket {
  // Mission
  uint32_t counter;
  uint32_t time_ms;

  // GPS
  float lat;
  float lon;
  float alt;
  float speed;
  uint8_t sat;

  // BME680
  float temp;
  float humidity;
  float pressure;
  float gas_kohm;

  // MPU6050 acceleration
  int16_t accel_x_cms2;
  int16_t accel_y_cms2;
  int16_t accel_z_cms2;
  int16_t accel_total_cms2;

  // Mission state
  uint8_t mode;
};

static_assert(
  sizeof(TelemetryPacket) == 50,
  "TelemetryPacket must be exactly 50 bytes"
);


// ======================================================
// ESP-NOW
// ======================================================

#define ESPNOW_CHANNEL 1

// Broadcast until a fixed ground-station MAC is used.
uint8_t groundStationAddress[] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

bool espNowReady = false;

uint32_t espNowQueued = 0;
uint32_t espNowErrors = 0;


// ======================================================
// Camera timing
// ======================================================

unsigned long lastPhotoMs = 0;
const unsigned long photoIntervalMs = 5000;

int photoNumber = 0;


// ======================================================
// ESP-NOW initialization
// ======================================================

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t channelResult =
    esp_wifi_set_channel(
      ESPNOW_CHANNEL,
      WIFI_SECOND_CHAN_NONE
    );

  if (channelResult != ESP_OK) {
    Serial.print("Failed to set ESP-NOW channel: ");
    Serial.println(channelResult);
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return false;
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    groundStationAddress,
    6
  );

  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(
      "Failed to add ESP-NOW ground-station peer"
    );
    return false;
  }

  Serial.print("ESP-NOW ready on channel ");
  Serial.println(ESPNOW_CHANNEL);

  Serial.print("ESP32-CAM STA MAC: ");
  Serial.println(WiFi.macAddress());

  espNowReady = true;
  return true;
}


// ======================================================
// ESP-NOW transmission
// ======================================================

bool sendPacketEspNow(
  const uint8_t *data,
  size_t length
) {
  if (!espNowReady) {
    return false;
  }

  esp_err_t result =
    esp_now_send(
      groundStationAddress,
      data,
      length
    );

  if (result != ESP_OK) {
    espNowErrors++;

    Serial.print("ESP-NOW send error: ");
    Serial.println(result);

    return false;
  }

  espNowQueued++;
  return true;
}


// ======================================================
// Telemetry CSV logging
//
// Radio values remain compact (cm/s^2 as int16_t),
// but CSV stores acceleration in m/s^2 for readability.
// ======================================================

bool appendTelemetryRow(
  const TelemetryPacket &p
) {
  File file =
    SD_MMC.open(
      telemetryLogPath,
      FILE_APPEND
    );

  if (!file) {
    Serial.println(
      "Failed to open telemetry.csv for appending"
    );
    return false;
  }

  file.print(p.counter);
  file.print(",");

  file.print(p.time_ms);
  file.print(",");

  file.print(p.lat, 6);
  file.print(",");

  file.print(p.lon, 6);
  file.print(",");

  file.print(p.alt, 2);
  file.print(",");

  file.print(p.speed, 2);
  file.print(",");

  file.print(p.sat);
  file.print(",");

  file.print(p.temp, 2);
  file.print(",");

  file.print(p.humidity, 2);
  file.print(",");

  file.print(p.pressure, 2);
  file.print(",");

  file.print(p.gas_kohm, 2);
  file.print(",");

  file.print(
    p.accel_x_cms2 / 100.0f,
    2
  );
  file.print(",");

  file.print(
    p.accel_y_cms2 / 100.0f,
    2
  );
  file.print(",");

  file.print(
    p.accel_z_cms2 / 100.0f,
    2
  );
  file.print(",");

  file.print(
    p.accel_total_cms2 / 100.0f,
    2
  );
  file.print(",");

  file.println(p.mode);

  file.close();

  return true;
}


// ======================================================
// Setup
// ======================================================

void setup() {
  // Larger RX buffer helps absorb UART telemetry
  // while camera/SD operations are busy.
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.setTimeout(50);
  Serial.println();

  // ----------------------------------------------------
  // Camera configuration
  // ----------------------------------------------------

  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err =
    esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf(
      "Camera init failed with error 0x%x\n",
      err
    );
    return;
  }

  sensor_t *s =
    esp_camera_sensor_get();

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  // Keep the existing QVGA operating resolution.
  s->set_framesize(
    s,
    FRAMESIZE_QVGA
  );

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  // ----------------------------------------------------
  // SD
  // ----------------------------------------------------

  Serial.println("Initialising SD card");

  if (!SD_MMC.begin()) {
    Serial.println(
      "Failed to initialise SD card!"
    );
    return;
  }

  uint8_t cardType =
    SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    Serial.println(
      "SD card slot appears to be empty!"
    );
    return;
  }

  // ----------------------------------------------------
  // Directories
  // ----------------------------------------------------

  if (!SD_MMC.exists("/logs")) {
    if (SD_MMC.mkdir("/logs")) {
      Serial.println("/logs created");
    } else {
      Serial.println(
        "Failed to create /logs"
      );
    }
  } else {
    Serial.println(
      "/logs already exists"
    );
  }

  if (!SD_MMC.exists("/photos")) {
    if (SD_MMC.mkdir("/photos")) {
      Serial.println("/photos created");
    } else {
      Serial.println(
        "Failed to create /photos"
      );
    }
  } else {
    Serial.println(
      "/photos already exists"
    );
  }

  // ----------------------------------------------------
  // Mission log
  // ----------------------------------------------------

  if (!SD_MMC.exists(missionLogPath)) {
    File file =
      SD_MMC.open(
        missionLogPath,
        FILE_WRITE
      );

    if (file) {
      file.println(
        "time_ms,photo_number,status"
      );
      file.close();

      Serial.println(
        "/logs/mission.csv created"
      );
    } else {
      Serial.println(
        "Failed to create /logs/mission.csv"
      );
    }
  } else {
    Serial.println(
      "/logs/mission.csv already exists"
    );
  }

  // ----------------------------------------------------
  // Unified telemetry log
  // ----------------------------------------------------

  if (!SD_MMC.exists(telemetryLogPath)) {
    File file =
      SD_MMC.open(
        telemetryLogPath,
        FILE_WRITE
      );

    if (file) {
      file.println(
        "counter,time_ms,"
        "lat,lon,alt,speed,sat,"
        "temp,humidity,pressure,gas_kohm,"
        "accel_x_ms2,accel_y_ms2,accel_z_ms2,"
        "accel_total_ms2,mode"
      );

      file.close();

      Serial.println(
        "/logs/telemetry.csv created"
      );
    } else {
      Serial.println(
        "Failed to create /logs/telemetry.csv"
      );
    }
  } else {
    Serial.println(
      "/logs/telemetry.csv already exists"
    );
  }

  // ----------------------------------------------------
  // ESP-NOW
  // ----------------------------------------------------

  if (!initEspNow()) {
    Serial.println(
      "ESP-NOW unavailable; SD logging "
      "and camera will continue"
    );
  }

  Serial.println();
  Serial.println(
    "ESP32-CAM ready for unified telemetry"
  );
  Serial.print(
    "Expected telemetry payload: "
  );
  Serial.print(
    sizeof(TelemetryPacket)
  );
  Serial.println(" bytes");
}


// ======================================================
// Process one validated UART payload
// ======================================================

void processTelemetryPayload(
  const uint8_t *buffer,
  uint8_t packetSize
) {
  if (
    packetSize != sizeof(TelemetryPacket)
  ) {
    Serial.print(
      "UART telemetry rejected: wrong size "
    );
    Serial.print(packetSize);
    Serial.print(" expected ");
    Serial.println(
      sizeof(TelemetryPacket)
    );

    uartRejectedFrames++;
    return;
  }

  TelemetryPacket telemetryPacket;

  memcpy(
    &telemetryPacket,
    buffer,
    sizeof(telemetryPacket)
  );

  // Mission mode is the only small enum field in this
  // simplified packet, so use it as a basic sanity check.
  if (telemetryPacket.mode > 2) {
    Serial.print(
      "UART telemetry rejected: invalid mode "
    );
    Serial.println(
      telemetryPacket.mode
    );

    uartRejectedFrames++;
    return;
  }

  bool saved =
    appendTelemetryRow(
      telemetryPacket
    );

  bool forwarded =
    sendPacketEspNow(
      buffer,
      packetSize
    );

  Serial.println();
  Serial.println(
    "===== TELEMETRY UART OK ====="
  );

  Serial.print("Counter: ");
  Serial.println(
    telemetryPacket.counter
  );

  Serial.print("Mission time ms: ");
  Serial.println(
    telemetryPacket.time_ms
  );

  Serial.print("GPS: ");
  Serial.print(
    telemetryPacket.lat,
    6
  );
  Serial.print(", ");
  Serial.println(
    telemetryPacket.lon,
    6
  );

  Serial.print("Altitude m: ");
  Serial.println(
    telemetryPacket.alt,
    2
  );

  Serial.print("Speed km/h: ");
  Serial.println(
    telemetryPacket.speed,
    2
  );

  Serial.print("Satellites: ");
  Serial.println(
    telemetryPacket.sat
  );

  Serial.print("Temperature C: ");
  Serial.println(
    telemetryPacket.temp,
    2
  );

  Serial.print("Humidity %: ");
  Serial.println(
    telemetryPacket.humidity,
    2
  );

  Serial.print("Pressure hPa: ");
  Serial.println(
    telemetryPacket.pressure,
    2
  );

  Serial.print("Gas kOhm: ");
  Serial.println(
    telemetryPacket.gas_kohm,
    2
  );

  Serial.print("Accel X m/s2: ");
  Serial.println(
    telemetryPacket.accel_x_cms2 /
    100.0f,
    2
  );

  Serial.print("Accel Y m/s2: ");
  Serial.println(
    telemetryPacket.accel_y_cms2 /
    100.0f,
    2
  );

  Serial.print("Accel Z m/s2: ");
  Serial.println(
    telemetryPacket.accel_z_cms2 /
    100.0f,
    2
  );

  Serial.print("Accel total m/s2: ");
  Serial.println(
    telemetryPacket.accel_total_cms2 /
    100.0f,
    2
  );

  Serial.print("Mode: ");
  Serial.println(
    telemetryPacket.mode
  );

  Serial.print("SD saved: ");
  Serial.println(
    saved
      ? "YES"
      : "NO"
  );

  Serial.print("ESP-NOW queued: ");
  Serial.println(
    forwarded
      ? "YES"
      : "NO"
  );
}


// ======================================================
// UART parser
// ======================================================

void resetUartParser() {
  uartRxState =
    UART_WAIT_SYNC_1;

  uartRxLength = 0;
  uartRxIndex = 0;
  uartRxChecksum = 0;
}


void handleTelemetry() {
  while (Serial.available() > 0) {
    uint8_t b =
      (uint8_t)Serial.read();

    switch (uartRxState) {

      case UART_WAIT_SYNC_1:
        if (b == UART_SYNC_1) {
          uartRxState =
            UART_WAIT_SYNC_2;
        }
        break;


      case UART_WAIT_SYNC_2:
        if (b == UART_SYNC_2) {
          uartRxState =
            UART_WAIT_LENGTH;
        }

        else if (b == UART_SYNC_1) {
          uartRxState =
            UART_WAIT_SYNC_2;
        }

        else {
          uartRxState =
            UART_WAIT_SYNC_1;
        }
        break;


      case UART_WAIT_LENGTH:
        uartRxLength = b;
        uartRxIndex = 0;
        uartRxChecksum = b;

        if (
          uartRxLength == 0 ||
          uartRxLength > UART_MAX_PAYLOAD
        ) {
          Serial.print(
            "UART invalid frame length: "
          );
          Serial.println(
            uartRxLength
          );

          uartRejectedFrames++;
          resetUartParser();
        }

        else if (
          uartRxLength !=
          sizeof(TelemetryPacket)
        ) {
          Serial.print(
            "UART unexpected payload length: "
          );
          Serial.println(
            uartRxLength
          );

          uartRejectedFrames++;
          resetUartParser();
        }

        else {
          uartRxState =
            UART_READ_PAYLOAD;
        }
        break;


      case UART_READ_PAYLOAD:
        uartRxBuffer[
          uartRxIndex++
        ] = b;

        uartRxChecksum ^= b;

        if (
          uartRxIndex >=
          uartRxLength
        ) {
          uartRxState =
            UART_WAIT_CHECKSUM;
        }
        break;


      case UART_WAIT_CHECKSUM:
        if (b == uartRxChecksum) {
          uartFramesOk++;

          processTelemetryPayload(
            uartRxBuffer,
            uartRxLength
          );
        }

        else {
          uartChecksumErrors++;

          Serial.print(
            "UART checksum error. expected=0x"
          );
          Serial.print(
            uartRxChecksum,
            HEX
          );

          Serial.print(
            " received=0x"
          );
          Serial.println(
            b,
            HEX
          );
        }

        resetUartParser();
        break;
    }
  }
}


// ======================================================
// Camera / photo logging
// Kept functionally the same as the working version.
// ======================================================

void takeAndSavePhoto() {
  Serial.println();
  Serial.println(
    "========== PHOTO DEBUG =========="
  );

  Serial.print("Photo number: ");
  Serial.println(photoNumber);

  Serial.print(
    "Free heap before capture: "
  );
  Serial.println(
    ESP.getFreeHeap()
  );

  Serial.print(
    "Free PSRAM before capture: "
  );
  Serial.println(
    ESP.getFreePsram()
  );

  unsigned long captureStartMs =
    millis();

  camera_fb_t *fb =
    esp_camera_fb_get();

  unsigned long captureElapsedMs =
    millis() - captureStartMs;

  if (!fb) {
    Serial.print(
      "PHOTO ERROR: esp_camera_fb_get() "
      "returned NULL after "
    );

    Serial.print(
      captureElapsedMs
    );

    Serial.println(" ms");
    Serial.println(
      "================================="
    );

    return;
  }

  Serial.print(
    "Capture OK in ms: "
  );
  Serial.println(
    captureElapsedMs
  );

  size_t capturedLen =
    fb->len;

  Serial.print(
    "JPEG bytes captured: "
  );
  Serial.println(
    capturedLen
  );

  Serial.print(
    "Frame width: "
  );
  Serial.println(
    fb->width
  );

  Serial.print(
    "Frame height: "
  );
  Serial.println(
    fb->height
  );

  String photoFileName =
    String(photoPrefix) +
    String(photoNumber) +
    ".jpg";

  Serial.print(
    "Target path: "
  );
  Serial.println(
    photoFileName
  );

  Serial.print(
    "/photos exists: "
  );
  Serial.println(
    SD_MMC.exists("/photos")
      ? "YES"
      : "NO"
  );

  File file =
    SD_MMC.open(
      photoFileName.c_str(),
      FILE_WRITE
    );

  if (!file) {
    Serial.println(
      "PHOTO ERROR: "
      "SD_MMC.open(FILE_WRITE) failed"
    );

    esp_camera_fb_return(fb);

    Serial.println(
      "================================="
    );

    return;
  }

  unsigned long writeStartMs =
    millis();

  size_t written =
    file.write(
      fb->buf,
      capturedLen
    );

  file.flush();

  size_t fileSizeBeforeClose =
    file.size();

  file.close();

  unsigned long writeElapsedMs =
    millis() - writeStartMs;

  Serial.print(
    "JPEG bytes requested: "
  );
  Serial.println(
    capturedLen
  );

  Serial.print(
    "JPEG bytes written: "
  );
  Serial.println(
    written
  );

  Serial.print(
    "File size before close: "
  );
  Serial.println(
    fileSizeBeforeClose
  );

  Serial.print(
    "SD write time ms: "
  );
  Serial.println(
    writeElapsedMs
  );

  esp_camera_fb_return(fb);

  if (written != capturedLen) {
    Serial.println(
      "PHOTO ERROR: incomplete JPEG write"
    );

    Serial.println(
      "================================="
    );

    return;
  }

  if (
    !SD_MMC.exists(
      photoFileName.c_str()
    )
  ) {
    Serial.println(
      "PHOTO ERROR: file does not exist "
      "after close"
    );

    Serial.println(
      "================================="
    );

    return;
  }

  File verifyFile =
    SD_MMC.open(
      photoFileName.c_str(),
      FILE_READ
    );

  if (!verifyFile) {
    Serial.println(
      "PHOTO ERROR: could not reopen "
      "saved JPG for verification"
    );

    Serial.println(
      "================================="
    );

    return;
  }

  size_t verifiedSize =
    verifyFile.size();

  verifyFile.close();

  Serial.print(
    "Verified JPG size: "
  );
  Serial.println(
    verifiedSize
  );

  if (
    verifiedSize != written ||
    verifiedSize == 0
  ) {
    Serial.println(
      "PHOTO ERROR: saved JPG size mismatch"
    );

    Serial.println(
      "================================="
    );

    return;
  }

  Serial.print(
    "PHOTO SAVED OK: "
  );
  Serial.println(
    photoFileName
  );

  ++photoNumber;

  Serial.println(
    "================================="
  );
}


// ======================================================
// Loop
// ======================================================

void loop() {
  // Telemetry gets priority.
  handleTelemetry();

  unsigned long now =
    millis();

  if (
    lastPhotoMs == 0 ||
    now - lastPhotoMs >=
    photoIntervalMs
  ) {
    lastPhotoMs = now;

    takeAndSavePhoto();

    // Drain UART immediately after photo/SD activity.
    handleTelemetry();
  }

  static unsigned long
    lastDebugStatsMs = 0;

  if (
    now - lastDebugStatsMs >=
    5000
  ) {
    lastDebugStatsMs = now;

    Serial.print(
      "UART stats | valid frames="
    );
    Serial.print(
      uartFramesOk
    );

    Serial.print(
      " checksum errors="
    );
    Serial.print(
      uartChecksumErrors
    );

    Serial.print(
      " rejected="
    );
    Serial.print(
      uartRejectedFrames
    );

    Serial.print(
      " ESP-NOW queued="
    );
    Serial.print(
      espNowQueued
    );

    Serial.print(
      " ESP-NOW errors="
    );
    Serial.print(
      espNowErrors
    );

    Serial.print(
      " rx buffered bytes="
    );
    Serial.println(
      Serial.available()
    );
  }

  delay(1);
}