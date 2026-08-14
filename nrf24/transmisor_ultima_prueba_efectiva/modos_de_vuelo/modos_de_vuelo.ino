#include <Wire.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include <Adafruit_VL53L0X.h>

HardwareSerial CamSerial(1);

// ======================================================
// UART framing ESP32 -> ESP32-CAM
// Frame: 0xAA 0x55 | payload_length | payload | XOR checksum
// ======================================================

const uint8_t UART_SYNC_1 = 0xAA;
const uint8_t UART_SYNC_2 = 0x55;

// ======================================================
// HERMES TELEMETRY PACKET
// ======================================================

struct __attribute__((packed)) TelemetryPacket {
  uint32_t counter;
  uint32_t time_ms;

  float lat;
  float lon;
  float alt;
  float speed;          // km/h
  uint8_t sat;

  float temp;           // deg C
  float humidity;       // %
  float pressure;       // hPa
  float gas_kohm;       // kOhm

  int16_t accel_x_cms2;
  int16_t accel_y_cms2;
  int16_t accel_z_cms2;
  int16_t accel_total_cms2;

  uint8_t mode;
};

static_assert(
  sizeof(TelemetryPacket) == 50,
  "TelemetryPacket must be exactly 50 bytes"
);

TelemetryPacket packet;

// ======================================================
// GPS
// ======================================================

#define RXD2 16
#define TXD2 17

TinyGPSPlus gps;

// ======================================================
// ESP32-CAM UART
// ======================================================

#define CAM_UART_RX 14
#define CAM_UART_TX 27

// ======================================================
// VL53L0X LiDAR
// Kept for local mission/debug use only.
// ======================================================

Adafruit_VL53L0X lidar = Adafruit_VL53L0X();

bool lidarOk = false;
bool lidarValid = false;
uint16_t lidarDistanceMm = 0;

// ======================================================
// MPU6050
// ======================================================

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

float accelTotal = NAN;

bool mpuValid = false;
bool mpuAngleInitialized = false;
bool mpuOk = false;

float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

const uint32_t MPU_INTERVAL_US = 20000; // 50 Hz
uint32_t lastMpuSampleUs = 0;

const float MPU_FILTER_TAU_S = 0.50f;

// ======================================================
// BME680
// ======================================================

Adafruit_BME680 bme;

float lastBmeTemp = NAN;
float lastPressure = NAN;
float lastHumidity = NAN;
float lastGas = NAN;

// ======================================================
// Last known GPS position
// ======================================================

float lastKnownLat = 0.0f;
float lastKnownLon = 0.0f;
float lastKnownAlt = 0.0f;
uint8_t lastKnownSat = 0;

bool hasLastKnownGps = false;

// ======================================================
// Timing
// ======================================================

unsigned long lastSendMs = 0;
const unsigned long sendIntervalMs = 750;

// ======================================================
// Mission modes
// ======================================================

enum MissionMode : uint8_t {
  MODE_PRELAUNCH = 0,
  MODE_DESCENT = 1,
  MODE_POST_IMPACT = 2
};

MissionMode currentMode = MODE_PRELAUNCH;

unsigned long missionStartMs = 0;
unsigned long modeStartMs = 0;

// ======================================================
// Function declarations
// ======================================================

void updateGPS();
void updateEnvironmentalSensors();
void updateLidar();

void updateMPU6050();
bool calibrateMPU6050(uint16_t samples = 500);
void printMPUReport();
void printMPUAngleReport();

float wrapAngle180(float angleDeg);
float complementaryAngle(float gyroPredictionDeg, float accelAngleDeg, float alpha);

int16_t accelToCms2(float accel_ms2);

size_t sendFramedUartPacket(const uint8_t *data, uint8_t length);
void sendTelemetryPacket();
void sendTelemetryCycle();

void handlePrelaunch();
void handleDescent();
void handlePostImpact();

void setMissionMode(MissionMode newMode);

// ======================================================
// Mission mode control
// ======================================================

void setMissionMode(MissionMode newMode) {
  currentMode = newMode;
  modeStartMs = millis();
}

// ======================================================
// Setup
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  missionStartMs = millis();
  modeStartMs = missionStartMs;
  currentMode = MODE_PRELAUNCH;

  CamSerial.setRxBufferSize(2048);
  CamSerial.begin(115200, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);

  Serial.println("ESP32-CAM UART link ready");

  Wire.begin(21, 22);

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  if (!lidar.begin()) {
    Serial.println("VL53L0X LiDAR not detected");
    lidarOk = false;
  } else {
    Serial.println("VL53L0X LiDAR detected");
    lidarOk = true;
  }

  if (!bme.begin(0x77)) {
    Serial.println("BME680 not found at 0x77, trying 0x76...");

    if (!bme.begin(0x76)) {
      Serial.println("BME680 not detected");
      while (1) {
        delay(1000);
      }
    }
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  Serial.println("BME680 detected");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected");
    mpuOk = false;
  } else {
    Serial.println("MPU6050 detected");
    mpuOk = true;

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(250);
    calibrateMPU6050();
    updateMPU6050();
  }

  packet.counter = 0;
  packet.time_ms = 0;

  Serial.println();
  Serial.println("======================================");
  Serial.println("Hermes onboard computer ready");
  Serial.print("TelemetryPacket size: ");
  Serial.print(sizeof(TelemetryPacket));
  Serial.println(" bytes");
  Serial.println("======================================");
}

// ======================================================
// Main loop
// ======================================================

void loop() {
  updateGPS();

  while (CamSerial.available()) {
    Serial.write(CamSerial.read());
  }

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastMpuSampleUs) >= MPU_INTERVAL_US) {
    updateMPU6050();
  }

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

// ======================================================
// Mission handlers
// ======================================================

void handlePrelaunch() {
  Serial.println();
  Serial.println("===== PRELAUNCH MODE =====");

  updateEnvironmentalSensors();
  printMPUReport();
  printMPUAngleReport();
  updateLidar();

  bool gpsOk = gps.location.isValid() && gps.location.age() < 3000;
  bool bmeOk = isfinite(lastBmeTemp) && isfinite(lastPressure) && isfinite(lastHumidity);

  Serial.print("GPS ready: ");
  Serial.println(gpsOk ? "YES" : "NO");

  Serial.print("BME680 ready: ");
  Serial.println(bmeOk ? "YES" : "NO");

  Serial.print("Prelaunch status: ");
  Serial.println((gpsOk && bmeOk) ? "READY" : "NOT READY");

  sendTelemetryCycle();
}

void handleDescent() {
  Serial.println();
  Serial.println("===== DESCENT MODE =====");

  updateEnvironmentalSensors();
  printMPUReport();
  printMPUAngleReport();
  updateLidar();

  sendTelemetryCycle();
}

void handlePostImpact() {
  Serial.println();
  Serial.println("===== POST IMPACT MODE =====");

  updateEnvironmentalSensors();
  printMPUReport();

  sendTelemetryCycle();
}

// ======================================================
// GPS
// ======================================================

void updateGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

// ======================================================
// BME680
// ======================================================

void updateEnvironmentalSensors() {
  if (bme.performReading()) {
    lastBmeTemp = bme.temperature;
    lastPressure = bme.pressure / 100.0f;
    lastHumidity = bme.humidity;
    lastGas = bme.gas_resistance / 1000.0f;
  } else {
    Serial.println("BME680 reading failed");
  }

  Serial.println();
  Serial.println("===== PAYLOAD SENSOR REPORT =====");

  Serial.print("GPS valid: ");
  Serial.println(gps.location.isValid() ? "yes" : "no");

  Serial.print("BME680 Temp C: ");
  Serial.println(lastBmeTemp);

  Serial.print("Pressure hPa: ");
  Serial.println(lastPressure);

  Serial.print("Humidity %: ");
  Serial.println(lastHumidity);

  Serial.print("Gas kOhm: ");
  Serial.println(lastGas);
}

// ======================================================
// LiDAR
// ======================================================

void updateLidar() {
  if (!lidarOk) {
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

  Serial.println();
  Serial.println("===== LIDAR REPORT =====");

  Serial.print("LiDAR valid: ");
  Serial.println(lidarValid ? "YES" : "NO");

  Serial.print("Distance mm: ");
  if (lidarValid) {
    Serial.println(lidarDistanceMm);
  } else {
    Serial.println("out of range / invalid");
  }
}

// ======================================================
// MPU6050 calibration
// ======================================================

bool calibrateMPU6050(uint16_t samples) {
  if (!mpuOk || samples == 0) {
    return false;
  }

  Serial.println();
  Serial.println("===== MPU6050 GYRO CALIBRATION =====");
  Serial.println("Keep the CanSat completely still...");

  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  for (uint16_t i = 0; i < samples; i++) {
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    sumX += gyroEvent.gyro.x;
    sumY += gyroEvent.gyro.y;
    sumZ += gyroEvent.gyro.z;

    delay(5);
  }

  gyroOffsetX = (float)(sumX / samples);
  gyroOffsetY = (float)(sumY / samples);
  gyroOffsetZ = (float)(sumZ / samples);

  mpuAngleInitialized = false;
  lastMpuSampleUs = micros();

  Serial.print("Gyro offset X rad/s: ");
  Serial.println(gyroOffsetX, 6);

  Serial.print("Gyro offset Y rad/s: ");
  Serial.println(gyroOffsetY, 6);

  Serial.print("Gyro offset Z rad/s: ");
  Serial.println(gyroOffsetZ, 6);

  Serial.println("MPU6050 gyro calibration complete");

  return true;
}

// ======================================================
// MPU6050 angle helpers
// ======================================================

float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }

  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }

  return angleDeg;
}

float complementaryAngle(float gyroPredictionDeg, float accelAngleDeg, float alpha) {
  float errorDeg = wrapAngle180(accelAngleDeg - gyroPredictionDeg);
  return wrapAngle180(gyroPredictionDeg + (1.0f - alpha) * errorDeg);
}

// ======================================================
// MPU6050 update
// ======================================================

void updateMPU6050() {
  if (!mpuOk) {
    mpuValid = false;
    return;
  }

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  uint32_t nowUs = micros();
  float dt = 0.0f;

  if (lastMpuSampleUs != 0) {
    dt = (uint32_t)(nowUs - lastMpuSampleUs) / 1000000.0f;
  }

  lastMpuSampleUs = nowUs;

  ax = accelEvent.acceleration.x;
  ay = accelEvent.acceleration.y;
  az = accelEvent.acceleration.z;

  gx = gyroEvent.gyro.x - gyroOffsetX;
  gy = gyroEvent.gyro.y - gyroOffsetY;
  gz = gyroEvent.gyro.z - gyroOffsetZ;

  accelTotal = sqrtf((ax * ax) + (ay * ay) + (az * az));

  float accRollDeg = atan2f(ay, az) * 180.0f / PI;
  float accPitchDeg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * 180.0f / PI;

  const float RAD_TO_DEG_F = 57.2957795f;

  float gyroXDegS = gx * RAD_TO_DEG_F;
  float gyroYDegS = gy * RAD_TO_DEG_F;
  float gyroZDegS = gz * RAD_TO_DEG_F;

  if (!mpuAngleInitialized) {
    rollDeg = accRollDeg;
    pitchDeg = accPitchDeg;
    yawDeg = 0.0f;
    mpuAngleInitialized = true;
  }
  else if (dt > 0.0f && dt <= 0.20f) {
    float alpha = MPU_FILTER_TAU_S / (MPU_FILTER_TAU_S + dt);

    float rollPrediction = wrapAngle180(rollDeg + gyroXDegS * dt);
    float pitchPrediction = wrapAngle180(pitchDeg + gyroYDegS * dt);

    rollDeg = complementaryAngle(rollPrediction, accRollDeg, alpha);
    pitchDeg = complementaryAngle(pitchPrediction, accPitchDeg, alpha);
    yawDeg = wrapAngle180(yawDeg + gyroZDegS * dt);
  }
  else if (dt > 0.20f) {
    rollDeg = accRollDeg;
    pitchDeg = accPitchDeg;
  }

  mpuValid =
    isfinite(ax) &&
    isfinite(ay) &&
    isfinite(az) &&
    isfinite(gx) &&
    isfinite(gy) &&
    isfinite(gz) &&
    isfinite(accelTotal);
}

// ======================================================
// MPU debug
// ======================================================

void printMPUReport() {
  Serial.println();
  Serial.println("===== MPU6050 REPORT =====");

  if (!mpuOk) {
    Serial.println("MPU6050 not available");
    return;
  }

  Serial.print("Accel m/s^2: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);

  Serial.print("Gyro corrected rad/s: ");
  Serial.print(gx, 4);
  Serial.print(", ");
  Serial.print(gy, 4);
  Serial.print(", ");
  Serial.println(gz, 4);

  Serial.print("Accel total m/s^2: ");
  Serial.println(accelTotal);
}

void printMPUAngleReport() {
  Serial.println();
  Serial.println("===== MPU ANGLE REPORT =====");

  Serial.print("Roll deg: ");
  Serial.println(rollDeg);

  Serial.print("Pitch deg: ");
  Serial.println(pitchDeg);

  Serial.print("Yaw deg (relative): ");
  Serial.println(yawDeg);

  Serial.print("MPU valid: ");
  Serial.println(mpuValid ? "YES" : "NO");
}

// ======================================================
// Acceleration conversion
// m/s^2 -> cm/s^2
// -32768 is reserved as invalid.
// ======================================================

int16_t accelToCms2(float accel_ms2) {
  if (!isfinite(accel_ms2)) {
    return -32768;
  }

  float scaled = accel_ms2 * 100.0f;

  if (scaled > 32767.0f) {
    scaled = 32767.0f;
  }

  if (scaled < -32767.0f) {
    scaled = -32767.0f;
  }

  return (int16_t)lroundf(scaled);
}

// ======================================================
// UART frame writer
// ======================================================

size_t sendFramedUartPacket(const uint8_t *data, uint8_t length) {
  uint8_t checksum = length;

  for (uint8_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }

  size_t totalSent = 0;

  totalSent += CamSerial.write(UART_SYNC_1);
  totalSent += CamSerial.write(UART_SYNC_2);
  totalSent += CamSerial.write(length);
  totalSent += CamSerial.write(data, length);
  totalSent += CamSerial.write(checksum);

  return totalSent;
}

// ======================================================
// Single telemetry packet
// ======================================================

void sendTelemetryPacket() {
  bool gpsOk = gps.location.isValid() && gps.location.age() < 3000;

  packet.counter++;
  packet.time_ms = millis() - missionStartMs;

  if (gpsOk) {
    packet.lat = (float)gps.location.lat();
    packet.lon = (float)gps.location.lng();

    packet.alt = gps.altitude.isValid()
      ? (float)gps.altitude.meters()
      : 0.0f;

    packet.speed = gps.speed.isValid()
      ? (float)gps.speed.kmph()
      : 0.0f;

    packet.sat = gps.satellites.isValid()
      ? (uint8_t)gps.satellites.value()
      : 0;

    lastKnownLat = packet.lat;
    lastKnownLon = packet.lon;
    lastKnownAlt = packet.alt;
    lastKnownSat = packet.sat;
    hasLastKnownGps = true;
  }
  else if (hasLastKnownGps) {
    packet.lat = lastKnownLat;
    packet.lon = lastKnownLon;
    packet.alt = lastKnownAlt;
    packet.sat = lastKnownSat;
    packet.speed = 0.0f;
  }
  else {
    packet.lat = 0.0f;
    packet.lon = 0.0f;
    packet.alt = 0.0f;
    packet.speed = 0.0f;
    packet.sat = 0;
  }

  packet.temp = isfinite(lastBmeTemp) ? lastBmeTemp : -999.0f;
  packet.humidity = isfinite(lastHumidity) ? lastHumidity : -999.0f;
  packet.pressure = isfinite(lastPressure) ? lastPressure : -999.0f;
  packet.gas_kohm = isfinite(lastGas) ? lastGas : -999.0f;

  packet.accel_x_cms2 = accelToCms2(ax);
  packet.accel_y_cms2 = accelToCms2(ay);
  packet.accel_z_cms2 = accelToCms2(az);
  packet.accel_total_cms2 = accelToCms2(accelTotal);

  packet.mode = (uint8_t)currentMode;

  size_t bytesSent = sendFramedUartPacket(
    (const uint8_t *)&packet,
    sizeof(packet)
  );

  Serial.println();
  Serial.println("===== TELEMETRY PACKET =====");

  Serial.print("Counter: ");
  Serial.println(packet.counter);

  Serial.print("Mission time ms: ");
  Serial.println(packet.time_ms);

  Serial.print("Lat: ");
  Serial.println(packet.lat, 6);

  Serial.print("Lon: ");
  Serial.println(packet.lon, 6);

  Serial.print("Alt m: ");
  Serial.println(packet.alt);

  Serial.print("Speed km/h: ");
  Serial.println(packet.speed);

  Serial.print("Satellites: ");
  Serial.println(packet.sat);

  Serial.print("Temperature C: ");
  Serial.println(packet.temp);

  Serial.print("Humidity %: ");
  Serial.println(packet.humidity);

  Serial.print("Pressure hPa: ");
  Serial.println(packet.pressure);

  Serial.print("Gas kOhm: ");
  Serial.println(packet.gas_kohm);

  Serial.print("Accel X cm/s^2: ");
  Serial.println(packet.accel_x_cms2);

  Serial.print("Accel Y cm/s^2: ");
  Serial.println(packet.accel_y_cms2);

  Serial.print("Accel Z cm/s^2: ");
  Serial.println(packet.accel_z_cms2);

  Serial.print("Accel total cm/s^2: ");
  Serial.println(packet.accel_total_cms2);

  Serial.print("Mission mode: ");
  Serial.println(packet.mode);

  Serial.print("Payload bytes: ");
  Serial.println(sizeof(packet));

  Serial.print("UART frame bytes sent: ");
  Serial.println(bytesSent);
}

void sendTelemetryCycle() {
  sendTelemetryPacket();
}