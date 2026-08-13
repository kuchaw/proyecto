#include "esp_camera.h"
#include "SD_MMC.h"
#include "FS.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"

const char * photoPrefix = "/photos/photo_";
const char * missionLogPath = "/logs/mission.csv";
const char * eventLogPath = "/logs/events.txt";
const char *attitudeLogPath = "/logs/attitude.csv";

#define PACKET_ENVIRONMENT 1
#define PACKET_ATTITUDE    2
#define PACKET_VERSION     1

// =========================
// UART framing ESP32 -> ESP32-CAM
// Frame: 0xAA 0x55 | payload_length | payload | XOR checksum
// =========================
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

// ESP-NOW
#define ESPNOW_CHANNEL 1

// Broadcast is used until the ground-station MAC is fixed.
// Both devices must use the same WiFi channel.
uint8_t groundStationAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool espNowReady = false;
unsigned long lastPhotoMs = 0;
const unsigned long photoIntervalMs = 5000;

int photoNumber = 0;
/*struct TelemetryPacket {
  uint32_t counter;
  float lat;
  float lon;
  float alt;
  float temp;
  float pressure;
  float humidity;
  uint8_t sat;
  uint8_t padding[3];
};*/

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

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t channelResult = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
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
  memcpy(peerInfo.peer_addr, groundStationAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW ground-station peer");
    return false;
  }

  Serial.print("ESP-NOW ready on channel ");
  Serial.println(ESPNOW_CHANNEL);
  Serial.print("ESP32-CAM STA MAC: ");
  Serial.println(WiFi.macAddress());

  espNowReady = true;
  return true;
}

bool sendPacketEspNow(const uint8_t *data, size_t length) {
  if (!espNowReady) {
    return false;
  }

  esp_err_t result = esp_now_send(groundStationAddress, data, length);

  if (result != ESP_OK) {
    Serial.print("ESP-NOW send error: ");
    Serial.println(result);
    return false;
  }

  return true;
}

bool appendTelemetryRow(const TelemetryPacket &p) {
  File file = SD_MMC.open("/logs/telemetry.csv", FILE_APPEND);

  if (!file) {
    Serial.println("Failed to open telemetry.csv for appending");
    return false;
  }

  file.print(p.counter);
  file.print(",");

  file.print(p.lat, 6);
  file.print(",");

  file.print(p.lon, 6);
  file.print(",");

  file.print(p.alt, 2);
  file.print(",");

  file.print(p.temp, 2);
  file.print(",");

  file.print(p.pressure, 2);
  file.print(",");

  file.print(p.humidity, 2);
  file.print(",");

  file.println(p.sat);

  file.close();

  return true;
}

bool appendAttitudeRow(const TelemetryAttitudePacket &p) {
  File file = SD_MMC.open(attitudeLogPath, FILE_APPEND);

  if (!file) {
    Serial.println("Failed to open attitude.csv for appending");
    return false;
  }

  file.print(p.packetType);
  file.print(",");

  file.print(p.version);
  file.print(",");

  file.print(p.counter);
  file.print(",");

  file.print(p.time_ms);
  file.print(",");

  file.print(p.lidar_mm);
  file.print(",");

  file.print(p.roll_deg10 / 10.0, 1);
  file.print(",");

  file.print(p.pitch_deg10 / 10.0, 1);
  file.print(",");

  file.print(p.yaw_deg10 / 10.0, 1);
  file.print(",");

  file.print(p.mode);
  file.print(",");

  file.print(p.lidar_status);
  file.print(",");

  file.println(p.mpu_status);

  file.close();
  return true;
}

void setup() {
  // Larger RX buffer helps absorb telemetry while camera/SD operations are busy.
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.setTimeout(50);
  Serial.println();

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

  // camera init
  esp_err_t err = esp_camera_init( & config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s -> id.PID == OV3660_PID) {
    s -> set_vflip(s, 1); // flip it back
    s -> set_brightness(s, 1); // up the brightness just a bit
    s -> set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s -> set_framesize(s, FRAMESIZE_QVGA);

  #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s -> set_vflip(s, 1);
  s -> set_hmirror(s, 1);
  #endif

  Serial.println("Initialising SD card");
  if (!SD_MMC.begin()) {
    Serial.println("Failed to initialise SD card!");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("SD card slot appears to be empty!");
    return;
  }
  if (!SD_MMC.exists("/logs")) {
    if (SD_MMC.mkdir("/logs")) {
      Serial.println("/logs created");
  }   else {
      Serial.println("Failed to create /logs");
  }
}   else {
    Serial.println("/logs already exists");
}
  if (!SD_MMC.exists("/photos")) {
    if (SD_MMC.mkdir("/photos")) {
      Serial.println("/photos created");
  }   else {
      Serial.println("Failed to create /photos");
  }
}   else {
    Serial.println("/photos already exists");
}
  if(!SD_MMC.exists("/logs/mission.csv")){
    File file = SD_MMC.open("/logs/mission.csv", FILE_WRITE);

    if (file){
      file.print("time_ms, photo_number, status");
      file.close();
      Serial.println("/logs/mission.csv fue creado");
    }else{
      Serial.print("No se logro crear el archivo logs/mission.csv");
    }
  }else{
    Serial.println("/logs/mission.csv ya existe");
  }

  if (!SD_MMC.exists("/logs/telemetry.csv")) {
  File file = SD_MMC.open("/logs/telemetry.csv", FILE_WRITE);

  if (file) {
    file.println("counter,lat,lon,alt,temp,pressure,humidity,sat");
    file.close();
    } 
  }

  if (!SD_MMC.exists(attitudeLogPath)) {
  File file = SD_MMC.open(attitudeLogPath, FILE_WRITE);

  if (file) {
    file.println("packetType,version,counter,time_ms,lidar_mm,roll_deg,pitch_deg,yaw_deg,mode,lidar_status,mpu_status");
    file.close();
  }
}

  if (!initEspNow()) {
    Serial.println("ESP-NOW unavailable; SD logging and camera will continue");
  }
}

void processTelemetryPayload(const uint8_t *buffer, uint8_t packetSize) {
  if (packetSize == 0) {
    uartRejectedFrames++;
    return;
  }

  uint8_t packetType = buffer[0];

  if (packetType == PACKET_ENVIRONMENT) {
    if (packetSize != sizeof(TelemetryPacket)) {
      Serial.print("UART CORE rejected: wrong size ");
      Serial.println(packetSize);
      uartRejectedFrames++;
      return;
    }

    TelemetryPacket telemetryPacket;
    memcpy(&telemetryPacket, buffer, sizeof(telemetryPacket));

    if (telemetryPacket.version != PACKET_VERSION) {
      Serial.print("UART CORE rejected: version ");
      Serial.println(telemetryPacket.version);
      uartRejectedFrames++;
      return;
    }

    bool saved = appendTelemetryRow(telemetryPacket);

    Serial.print("UART CORE OK counter=");
    Serial.print(telemetryPacket.counter);
    Serial.print(" saved=");
    Serial.println(saved ? "YES" : "NO");
  }
  else if (packetType == PACKET_ATTITUDE) {
    if (packetSize != sizeof(TelemetryAttitudePacket)) {
      Serial.print("UART ATTITUDE rejected: wrong size ");
      Serial.println(packetSize);
      uartRejectedFrames++;
      return;
    }

    TelemetryAttitudePacket attitudePacket;
    memcpy(&attitudePacket, buffer, sizeof(attitudePacket));

    if (attitudePacket.version != PACKET_VERSION) {
      Serial.print("UART ATTITUDE rejected: version ");
      Serial.println(attitudePacket.version);
      uartRejectedFrames++;
      return;
    }

    // Reject impossible status values instead of writing corrupted rows to SD.
    if (attitudePacket.mode > 2 ||
        attitudePacket.lidar_status > 1 ||
        attitudePacket.mpu_status > 1) {
      Serial.print("UART ATTITUDE rejected: invalid status fields mode=");
      Serial.print(attitudePacket.mode);
      Serial.print(" lidar_status=");
      Serial.print(attitudePacket.lidar_status);
      Serial.print(" mpu_status=");
      Serial.println(attitudePacket.mpu_status);
      uartRejectedFrames++;
      return;
    }

    bool saved = appendAttitudeRow(attitudePacket);

    Serial.print("UART ATTITUDE OK counter=");
    Serial.print(attitudePacket.counter);
    Serial.print(" roll=");
    Serial.print(attitudePacket.roll_deg10 / 10.0f, 1);
    Serial.print(" pitch=");
    Serial.print(attitudePacket.pitch_deg10 / 10.0f, 1);
    Serial.print(" yaw=");
    Serial.print(attitudePacket.yaw_deg10 / 10.0f, 1);
    Serial.print(" saved=");
    Serial.println(saved ? "YES" : "NO");
  }
  else {
    Serial.print("UART frame rejected: unknown packet type ");
    Serial.println(packetType);
    uartRejectedFrames++;
    return;
  }

  // Forward exactly the validated telemetry payload received by UART.
  if (!sendPacketEspNow(buffer, packetSize)) {
    Serial.println("ESP-NOW forwarding failed");
  }
}

void resetUartParser() {
  uartRxState = UART_WAIT_SYNC_1;
  uartRxLength = 0;
  uartRxIndex = 0;
  uartRxChecksum = 0;
}

void handleTelemetry() {
  // Non-blocking byte-by-byte state machine. It can recover automatically
  // if the ESP32-CAM starts listening in the middle of a UART packet.
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    switch (uartRxState) {
      case UART_WAIT_SYNC_1:
        if (b == UART_SYNC_1) {
          uartRxState = UART_WAIT_SYNC_2;
        }
        break;

      case UART_WAIT_SYNC_2:
        if (b == UART_SYNC_2) {
          uartRxState = UART_WAIT_LENGTH;
        } else if (b == UART_SYNC_1) {
          // Could already be the first byte of the next sync sequence.
          uartRxState = UART_WAIT_SYNC_2;
        } else {
          uartRxState = UART_WAIT_SYNC_1;
        }
        break;

      case UART_WAIT_LENGTH:
        uartRxLength = b;
        uartRxIndex = 0;
        uartRxChecksum = b;

        if (uartRxLength == 0 || uartRxLength > UART_MAX_PAYLOAD) {
          Serial.print("UART invalid frame length: ");
          Serial.println(uartRxLength);
          uartRejectedFrames++;
          resetUartParser();
        } else {
          uartRxState = UART_READ_PAYLOAD;
        }
        break;

      case UART_READ_PAYLOAD:
        uartRxBuffer[uartRxIndex++] = b;
        uartRxChecksum ^= b;

        if (uartRxIndex >= uartRxLength) {
          uartRxState = UART_WAIT_CHECKSUM;
        }
        break;

      case UART_WAIT_CHECKSUM:
        if (b == uartRxChecksum) {
          uartFramesOk++;
          processTelemetryPayload(uartRxBuffer, uartRxLength);
        } else {
          uartChecksumErrors++;
          Serial.print("UART checksum error. expected=0x");
          Serial.print(uartRxChecksum, HEX);
          Serial.print(" received=0x");
          Serial.println(b, HEX);
        }

        resetUartParser();
        break;
    }
  }
}

void takeAndSavePhoto() {
  Serial.println();
  Serial.println("========== PHOTO DEBUG ==========");
  Serial.print("Photo number: ");
  Serial.println(photoNumber);
  Serial.print("Free heap before capture: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("Free PSRAM before capture: ");
  Serial.println(ESP.getFreePsram());

  unsigned long captureStartMs = millis();
  camera_fb_t *fb = esp_camera_fb_get();
  unsigned long captureElapsedMs = millis() - captureStartMs;

  if (!fb) {
    Serial.print("PHOTO ERROR: esp_camera_fb_get() returned NULL after ");
    Serial.print(captureElapsedMs);
    Serial.println(" ms");
    Serial.println("=================================");
    return;
  }

  Serial.print("Capture OK in ms: ");
  Serial.println(captureElapsedMs);
  size_t capturedLen = fb->len;

  Serial.print("JPEG bytes captured: ");
  Serial.println(capturedLen);
  Serial.print("Frame width: ");
  Serial.println(fb->width);
  Serial.print("Frame height: ");
  Serial.println(fb->height);

  String photoFileName = String(photoPrefix) + String(photoNumber) + ".jpg";
  Serial.print("Target path: ");
  Serial.println(photoFileName);
  Serial.print("/photos exists: ");
  Serial.println(SD_MMC.exists("/photos") ? "YES" : "NO");

  File file = SD_MMC.open(photoFileName.c_str(), FILE_WRITE);

  if (!file) {
    Serial.println("PHOTO ERROR: SD_MMC.open(FILE_WRITE) failed");
    esp_camera_fb_return(fb);
    Serial.println("=================================");
    return;
  }

  unsigned long writeStartMs = millis();
  size_t written = file.write(fb->buf, capturedLen);
  file.flush();
  size_t fileSizeBeforeClose = file.size();
  file.close();
  unsigned long writeElapsedMs = millis() - writeStartMs;

  Serial.print("JPEG bytes requested: ");
  Serial.println(capturedLen);
  Serial.print("JPEG bytes written: ");
  Serial.println(written);
  Serial.print("File size before close: ");
  Serial.println(fileSizeBeforeClose);
  Serial.print("SD write time ms: ");
  Serial.println(writeElapsedMs);

  esp_camera_fb_return(fb);

  if (written != capturedLen) {
    Serial.println("PHOTO ERROR: incomplete JPEG write");
    Serial.println("=================================");
    return;
  }

  if (!SD_MMC.exists(photoFileName.c_str())) {
    Serial.println("PHOTO ERROR: file does not exist after close");
    Serial.println("=================================");
    return;
  }

  File verifyFile = SD_MMC.open(photoFileName.c_str(), FILE_READ);
  if (!verifyFile) {
    Serial.println("PHOTO ERROR: could not reopen saved JPG for verification");
    Serial.println("=================================");
    return;
  }

  size_t verifiedSize = verifyFile.size();
  verifyFile.close();

  Serial.print("Verified JPG size: ");
  Serial.println(verifiedSize);

  if (verifiedSize != written || verifiedSize == 0) {
    Serial.println("PHOTO ERROR: saved JPG size mismatch");
    Serial.println("=================================");
    return;
  }

  Serial.print("PHOTO SAVED OK: ");
  Serial.println(photoFileName);
  ++photoNumber;
  Serial.println("=================================");
}

void loop() {
  // Always drain UART first so telemetry has priority.
  handleTelemetry();

  unsigned long now = millis();
  if (lastPhotoMs == 0 || now - lastPhotoMs >= photoIntervalMs) {
    lastPhotoMs = now;
    takeAndSavePhoto();

    // A photo capture/write can take some time; immediately drain any
    // telemetry that arrived while the camera/SD were busy.
    handleTelemetry();
  }

  static unsigned long lastDebugStatsMs = 0;
  if (now - lastDebugStatsMs >= 5000) {
    lastDebugStatsMs = now;
    Serial.print("UART stats | valid frames=");
    Serial.print(uartFramesOk);
    Serial.print(" checksum errors=");
    Serial.print(uartChecksumErrors);
    Serial.print(" rejected=");
    Serial.print(uartRejectedFrames);
    Serial.print(" rx buffered bytes=");
    Serial.println(Serial.available());
  }

  delay(1);
}