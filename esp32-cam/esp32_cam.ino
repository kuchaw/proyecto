#include "esp_camera.h"
#include "SD_MMC.h"
#include "FS.h"

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
#define UART_PACKET_SIZE   32

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
}

void handleTelemetry() {
  int packetsHandled = 0;
  const int maxPacketsPerLoop = 5;

  while (Serial.available() >= UART_PACKET_SIZE && packetsHandled < maxPacketsPerLoop) {
    uint8_t buffer[UART_PACKET_SIZE];

    int bytesRead = Serial.readBytes(
      (char *)buffer,
      UART_PACKET_SIZE
    );

    if (bytesRead == UART_PACKET_SIZE) {
      uint8_t packetType = buffer[0];

      if (packetType == PACKET_ATTITUDE) {
        TelemetryAttitudePacket attitudePacket;
        memcpy(&attitudePacket, buffer, sizeof(attitudePacket));
        appendAttitudeRow(attitudePacket);
      }

      else if (packetType == PACKET_ENVIRONMENT) {
        TelemetryPacket telemetryPacket;
        memcpy(&telemetryPacket, buffer, sizeof(telemetryPacket));
        appendTelemetryRow(telemetryPacket);
      }

      else {
        Serial.print("Unknown packet type: ");
        Serial.println(packetType);
      }
    }

    packetsHandled++;
  }
}

void loop() {
  handleTelemetry();

  camera_fb_t * fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed");
    delay(5000);
    return;
  }

  String photoFileName = String(photoPrefix) + String(photoNumber) + ".jpg";
  Serial.printf("Picture file name: %s\n", photoFileName.c_str());

  File file = SD_MMC.open(photoFileName.c_str(), FILE_WRITE);

  if (!file) {
    Serial.println("Failed to open file in writing mode");
  } else {
    file.write(fb->buf, fb->len);
    file.close();

    Serial.printf("Saved file to path: %s\n", photoFileName.c_str());
    ++photoNumber;
  }

  esp_camera_fb_return(fb);

  handleTelemetry();

  delay(5000);
}