
#
#include <Wire.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include <Adafruit_VL53L0X.h>
HardwareSerial CamSerial(1);

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
  uint8_t packetType;      // PACKET_ATTITUDE
  uint8_t version;         // packet format version

  uint32_t counter;        // packet counter
  uint32_t time_ms;        // time since mission start

  uint16_t lidar_mm;       // LiDAR distance in millimeters

  int16_t roll_deg10;      // roll angle * 10
  int16_t pitch_deg10;     // pitch angle * 10
  int16_t yaw_deg10;       // yaw angle * 10

  uint8_t mode;            // mission mode
  uint8_t lidar_status;    // 0 invalid, 1 valid
  uint8_t mpu_status;      // 0 invalid, 1 valid

  uint8_t reserved[11];    // keep packet at 32 bytes
};

static_assert(sizeof(TelemetryAttitudePacket) == 32, "TelemetryAttitudePacket must be 32 bytes");


enum PacketType : uint8_t {
  PACKET_CORE = 1,
  PACKET_ATTITUDE = 2
};

const uint8_t PACKET_VERSION = 1;
TelemetryPacket packet;

// =========================
// GPS
// =========================
#define RXD2 16
#define TXD2 17

//cam

#define CAM_UART_RX 14
#define CAM_UART_TX 27

TinyGPSPlus gps;

// =========================
// VL53L0X LiDAR
// =========================
Adafruit_VL53L0X lidar = Adafruit_VL53L0X();

bool lidarOk = false;
bool lidarValid = false;
uint16_t lidarDistanceMm = 0;

// =========================
// MPU6050
// =========================
Adafruit_MPU6050 mpu;

float ax = NAN;
float ay = NAN;
float az = NAN;

float gx = NAN;
float gy = NAN;
float gz = NAN;

float rollDeg = NAN;
float pitchDeg = NAN;
float yawDeg = 0.0f;

bool mpuValid = false;
bool mpuAngleInitialized = false;

unsigned long lastMpuAngleMs = 0;

float accelTotal = NAN;
bool mpuOk = false;

// =========================
// BME680
// =========================
Adafruit_BME680 bme;

float lastBmeTemp = NAN;
float lastPressure = NAN;
float lastHumidity = NAN;
float lastGas = NAN;

//GPS variables globales
float lastKnownLat = 0.0f;
float lastKnownLon = 0.0f;
float lastKnownAlt = 0.0f;
uint8_t lastKnownSat = 0;
bool hasLastKnownGps = false;

// =========================
// Timing
// =========================
unsigned long lastSendMs = 0;
const unsigned long sendIntervalMs = 750;

// =========================
// Function declarations
// =========================
void updateGPS();
void updateEnvironmentalSensors();
void sendCorePacket();
void handlePrelaunch();
void handleDescent();
void handlePostImpact();
void updateMPU6050();
void updateLidar();
void sendTelemetryCycle();
// =========================
// Mission modes
// =========================
enum MissionMode : uint8_t {
  MODE_PRELAUNCH = 0,
  MODE_DESCENT = 1,
  MODE_POST_IMPACT = 2
};

MissionMode currentMode = MODE_PRELAUNCH;

unsigned long missionStartMs = 0;
unsigned long modeStartMs = 0;

void setMissionMode(MissionMode newMode) {
  currentMode = newMode;
  modeStartMs = millis();
}

void sendTelemetryCycle() {
  sendCorePacket();       // CORE packet -> ESP32-CAM by UART
  sendAttitudePacket();   // new LiDAR + MPU packet
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  missionStartMs = millis();
  modeStartMs = missionStartMs;
  currentMode = MODE_PRELAUNCH;

  CamSerial.begin(115200, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);
  Serial.print("ESP32-CAM UART logger ready");
  // I2C: default ESP32 pins SDA=21, SCL=22
  Wire.begin(21, 22);

  // GPS UART
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // LiDAR VL53L0X
  if (!lidar.begin()) {
    Serial.println("VL53L0X LiDAR not detected");
    lidarOk = false; 
  } else {
    Serial.println("VL53L0X LiDAR detected");
    lidarOk = true;
  }

  // BME680
  if (!bme.begin(0x77)) {
    Serial.println("BME680 not found at 0x77, trying 0x76...");

    if (!bme.begin(0x76)) {
      Serial.println("BME680 not detected");
      while (1);
    }
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  Serial.println("BME680 detected");

  // MPU6050
  if (!mpu.begin()) {
  Serial.println("MPU6050 not detected");
  mpuOk = false;
  } else {
    Serial.println("MPU6050 detected");
    mpuOk = true;

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  packet.packetType = PACKET_CORE;
  packet.version = PACKET_VERSION;
  packet.counter = 0;
  packet.time_ms = 0;
  packet.reserved[0] = 0;

  Serial.println("Payload ready: sensors + UART to ESP32-CAM");
}

void loop() {
  updateGPS();

  unsigned long now = millis();
if (now - lastSendMs >= sendIntervalMs) {
  lastSendMs = now;

  if (currentMode == MODE_PRELAUNCH) {
  handlePrelaunch();
} else if (currentMode == MODE_DESCENT) {
  handleDescent();
} else if (currentMode == MODE_POST_IMPACT) {
  handlePostImpact();
}
    }
  }



void handlePrelaunch() {
  Serial.println("\n===== PRELAUNCH MODE =====");

  updateEnvironmentalSensors();
  updateMPU6050();
  updateMPUAngles();
  updateLidar();

  

  bool gpsOk = gps.location.isValid() && gps.location.age() < 3000;
  bool bmeOk = !isnan(lastBmeTemp) && !isnan(lastPressure) && !isnan(lastHumidity);

  Serial.print("GPS ready: ");
  Serial.println(gpsOk ? "YES" : "NO");

  Serial.print("BME680 ready: ");
  Serial.println(bmeOk ? "YES" : "NO");

  

  Serial.print("Prelaunch status: ");
  if (gpsOk && bmeOk) {
    Serial.println("READY");
  } else {
    Serial.println("NOT READY");
  }
  sendTelemetryCycle();
}
void handleDescent() {
  Serial.println("\n===== DESCENT MODE =====");

  updateEnvironmentalSensors();
  updateMPU6050();

  sendTelemetryCycle();
}
void handlePostImpact() {
  Serial.println("\n===== POST IMPACT MODE =====");
  sendTelemetryCycle();
}

void updateGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

void updateLidar() {
  if (!lidarOk) {
    Serial.println("LiDAR not available");
    lidarValid = false;
    return;
  }

  VL53L0X_RangingMeasurementData_t measure;

  lidar.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    lidarDistanceMm = measure.RangeMilliMeter;
    lidarValid = true;
  } else {
    lidarValid = false;
  }

  Serial.println("\n===== LIDAR REPORT =====");

  Serial.print("LiDAR valid: ");
  Serial.println(lidarValid ? "YES" : "NO");

  Serial.print("Distance mm: ");
  if (lidarValid) {
    Serial.println(lidarDistanceMm);
  } else {
    Serial.println("out of range / invalid");
  }
}


void updateMPU6050() {
  if (!mpuOk) {
    Serial.println("MPU6050 not available");
    return;
  }

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  ax = accelEvent.acceleration.x;
  ay = accelEvent.acceleration.y;
  az = accelEvent.acceleration.z;

  gx = gyroEvent.gyro.x;
  gy = gyroEvent.gyro.y;
  gz = gyroEvent.gyro.z;

  accelTotal = sqrt((ax * ax) + (ay * ay) + (az * az));

  Serial.println("\n===== MPU6050 REPORT =====");

  Serial.print("Accel m/s^2: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);

  Serial.print("Gyro rad/s: ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.println(gz);

  Serial.print("Accel total m/s^2: ");
  Serial.println(accelTotal);
}

void updateEnvironmentalSensors() {
  if (bme.performReading()) {
    lastBmeTemp  = bme.temperature;
    lastPressure = bme.pressure / 100.0f;       // Pa -> hPa
    lastHumidity = bme.humidity;
    lastGas      = bme.gas_resistance / 1000.0f; // ohms -> KOhms
  } else {
    Serial.println("BME680 reading failed");
  }

  Serial.println("\n===== PAYLOAD SENSOR REPORT =====");

  Serial.print("GPS valid: ");
  Serial.println(gps.location.isValid() ? "yes" : "no");

  Serial.print("BME680 Temp: ");
  Serial.println(lastBmeTemp);

  Serial.print("Pressure hPa: ");
  Serial.println(lastPressure);

  Serial.print("Humidity %: ");
  Serial.println(lastHumidity);

  Serial.print("Gas KOhms: ");
  Serial.println(lastGas);
}
void updateMPUAngles() {
  if (!mpuOk) {
    mpuValid = false;
    return;
  }

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  unsigned long now = millis();

  float dt = 0.0f;
  if (lastMpuAngleMs != 0) {
    dt = (now - lastMpuAngleMs) / 1000.0f;
  }

  lastMpuAngleMs = now;

  float ax = accelEvent.acceleration.x;
  float ay = accelEvent.acceleration.y;
  float az = accelEvent.acceleration.z;

  float gyroXDegS = gyroEvent.gyro.x * 57.2957795f;
  float gyroYDegS = gyroEvent.gyro.y * 57.2957795f;
  float gyroZDegS = gyroEvent.gyro.z * 57.2957795f;

  float accRollDeg = atan2f(ay, az) * 180.0f / PI;
  float accPitchDeg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * 180.0f / PI;

  if (!mpuAngleInitialized || dt <= 0.0f || dt > 1.0f) {
    rollDeg = accRollDeg;
    pitchDeg = accPitchDeg;
    yawDeg = 0.0f;
    mpuAngleInitialized = true;
  } else {
    rollDeg = 0.98f * (rollDeg + gyroXDegS * dt) + 0.02f * accRollDeg;
    pitchDeg = 0.98f * (pitchDeg + gyroYDegS * dt) + 0.02f * accPitchDeg;
    yawDeg += gyroZDegS * dt;
  }

  while (yawDeg > 180.0f) yawDeg -= 360.0f;
  while (yawDeg < -180.0f) yawDeg += 360.0f;

  mpuValid = isfinite(rollDeg) && isfinite(pitchDeg) && isfinite(yawDeg);

  Serial.println("\n===== MPU ANGLE REPORT =====");

  Serial.print("Roll deg: ");
  Serial.println(rollDeg);

  Serial.print("Pitch deg: ");
  Serial.println(pitchDeg);

  Serial.print("Yaw deg: ");
  Serial.println(yawDeg);

  Serial.print("MPU valid: ");
  Serial.println(mpuValid ? "YES" : "NO");
}

int16_t angleToDeg10(float angleDeg) {
  if (!isfinite(angleDeg)) {
    return -32768;
  }

  float scaled = angleDeg * 10.0f;

  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32767.0f) scaled = -32767.0f;

  return (int16_t)lroundf(scaled);
}


void sendCorePacket() {

  bool gpsOk = gps.location.isValid() && gps.location.age() < 3000;

  packet.counter++;

if (gpsOk) {
  packet.lat = gps.location.lat();
  packet.lon = gps.location.lng();
  packet.alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
  packet.sat = gps.satellites.isValid() ? gps.satellites.value() : 0;

  lastKnownLat = packet.lat;
  lastKnownLon = packet.lon;
  lastKnownAlt = packet.alt;
  lastKnownSat = packet.sat;
  hasLastKnownGps = true;

  packet.reserved[0] = 1;  // current GPS valid
} 
else if (hasLastKnownGps) {
  packet.lat = lastKnownLat;
  packet.lon = lastKnownLon;
  packet.alt = lastKnownAlt;
  packet.sat = lastKnownSat;

  packet.reserved[0] = 2;  // using last known GPS
} 
else {
  packet.lat = 0.0f;
  packet.lon = 0.0f;
  packet.alt = 0.0f;
  packet.sat = 0;

  packet.reserved[0] = 0;  // no GPS available
}

  packet.temp = isnan(lastBmeTemp) ? -999.0f : lastBmeTemp;
  packet.pressure = isnan(lastPressure) ? -999.0f : lastPressure;
  packet.humidity = isnan(lastHumidity) ? -999.0f : lastHumidity;

  packet.packetType = PACKET_CORE;
  packet.version = PACKET_VERSION;
  packet.time_ms = millis() - missionStartMs;

  size_t bytesSent = CamSerial.write((uint8_t *)&packet, sizeof(packet));

  Serial.print("UART CORE packet ");
  Serial.print(packet.counter);
  Serial.print(" size=");
  Serial.print(sizeof(packet));
  Serial.print(" bytes sent=");
  Serial.println(bytesSent);
}

void sendAttitudePacket() {
  TelemetryAttitudePacket attitude;

  attitude.packetType = PACKET_ATTITUDE;
  attitude.version = PACKET_VERSION;

  attitude.counter = packet.counter;
  attitude.time_ms = millis() - missionStartMs;

  attitude.lidar_mm = lidarValid ? lidarDistanceMm : 0;

  attitude.roll_deg10 = angleToDeg10(rollDeg);
  attitude.pitch_deg10 = angleToDeg10(pitchDeg);
  attitude.yaw_deg10 = angleToDeg10(yawDeg);

  attitude.mode = (uint8_t)currentMode;
  attitude.lidar_status = lidarValid ? 1 : 0;
  attitude.mpu_status = mpuValid ? 1 : 0;

  for (int i = 0; i < 11; i++) {
    attitude.reserved[i] = 0;
  }

  size_t bytesSent = CamSerial.write((uint8_t *)&attitude, sizeof(attitude));

  Serial.print("ATTITUDE packet ");
  Serial.print(attitude.counter);
  Serial.print(" time_ms=");
  Serial.print(attitude.time_ms);
  Serial.print(" lidar=");
  Serial.print(attitude.lidar_mm);
  Serial.print(" roll=");
  Serial.print(attitude.roll_deg10 / 10.0f);
  Serial.print(" pitch=");
  Serial.print(attitude.pitch_deg10 / 10.0f);
  Serial.print(" yaw=");
  Serial.print(attitude.yaw_deg10 / 10.0f);
  Serial.print(" bytes sent=");
  Serial.println(bytesSent);
}