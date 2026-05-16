#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>

// =====================================================
// nRF24 CONFIG
// =====================================================
#define NRF_CE 25
#define NRF_CSN 26

RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "GAAY1";

// =====================================================
// GPS CONFIG
// =====================================================
#define GPS_RXD2 16
#define GPS_TXD2 17

TinyGPSPlus gps;

// =====================================================
// I2C SENSORS
// =====================================================
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_BME680 bme;
Adafruit_MPU6050 mpu;

// =====================================================
// BATTERY ADC - DISABLED BY DEFAULT
// =====================================================
// Set ENABLE_BATTERY_ADC to 1 when battery divider is wired.
#define ENABLE_BATTERY_ADC 0
#define BATTERY_ADC_PIN 34

// Example divider:
// battery + ---- R1 ---- ADC ---- R2 ---- GND
// Vadc = Vbat * R2 / (R1 + R2)
const float BAT_R1 = 100000.0f;
const float BAT_R2 = 100000.0f;
const float ADC_REF = 3.3f;
const float ADC_MAX = 4095.0f;

// =====================================================
// TIMING
// =====================================================
const unsigned long SAMPLE_INTERVAL_MS = 1000;
unsigned long lastSampleMs = 0;

// =====================================================
// MISSION MODES
// =====================================================
enum MissionMode : uint8_t {
  MODE_PRE_LAUNCH  = 0,
  MODE_DESCENT     = 1,
  MODE_POST_IMPACT = 2
};

MissionMode currentMode = MODE_PRE_LAUNCH;

// =====================================================
// PACKET TYPES
// =====================================================
enum PacketType : uint8_t {
  PACKET_CORE = 1,
  PACKET_IMU  = 2
};

const uint8_t PACKET_VERSION = 1;

// =====================================================
// STATUS FLAGS
// =====================================================
enum StatusFlags : uint8_t {
  FLAG_GPS_VALID     = 1 << 0,
  FLAG_BME_VALID     = 1 << 1,
  FLAG_MPU_VALID     = 1 << 2,
  FLAG_BARO_VALID    = 1 << 3,
  FLAG_BATTERY_VALID = 1 << 4
};

// =====================================================
// SHARED PACKET HEADER
// 8 bytes
// =====================================================
struct __attribute__((packed)) PacketHeader {
  uint16_t counter;   // packet number
  uint8_t type;       // PACKET_CORE or PACKET_IMU
  uint8_t version;    // packet format version
  uint32_t time_ms;   // mission time
};

// =====================================================
// CORE PACKET - 32 bytes
// Navigation + environment
// =====================================================
struct __attribute__((packed)) TelemetryCorePacket {
  PacketHeader header;        // 8 bytes

  int32_t lat_e7;             // latitude  * 1e7
  int32_t lon_e7;             // longitude * 1e7

  int16_t gps_alt_dm;         // GPS altitude in decimeters
  int16_t baro_alt_dm;        // barometric altitude in decimeters

  int16_t speed_cms;          // GPS speed in cm/s
  int16_t temp_centi;         // temperature in °C * 100

  uint16_t pressure_x10;      // pressure in hPa * 10
  uint16_t humidity_centi;    // humidity in % * 100

  uint8_t sat;                // GPS satellites
  uint8_t mode;               // MissionMode

  uint16_t checksum;
};

static_assert(sizeof(TelemetryCorePacket) == 32, "TelemetryCorePacket must be 32 bytes");

// =====================================================
// IMU PACKET - 32 bytes
// Acceleration + gyro + system data
// =====================================================
struct __attribute__((packed)) TelemetryImuPacket {
  PacketHeader header;        // 8 bytes

  int16_t ax_mg;              // acceleration X in milli-g
  int16_t ay_mg;              // acceleration Y in milli-g
  int16_t az_mg;              // acceleration Z in milli-g

  int16_t gx_dps10;           // gyro X in deg/s * 10
  int16_t gy_dps10;           // gyro Y in deg/s * 10
  int16_t gz_dps10;           // gyro Z in deg/s * 10

  uint16_t accel_total_mg;    // total acceleration in milli-g
  uint16_t battery_mv;        // battery voltage in millivolts
  uint16_t gas_x10;           // gas resistance in kOhms * 10
  int16_t course_deg10;       // GPS course in degrees * 10

  uint8_t flags;              // sensor/system status
  uint8_t mode;               // MissionMode repeated

  uint16_t checksum;
};

static_assert(sizeof(TelemetryImuPacket) == 32, "TelemetryImuPacket must be 32 bytes");

// =====================================================
// SENSOR VALUES
// =====================================================
uint16_t packetCounter = 0;

float bmeTempC = NAN;
float pressureHpa = NAN;
float humidityPct = NAN;
float gasKOhms = NAN;

float gpsLat = NAN;
float gpsLon = NAN;
float gpsAltM = NAN;
float gpsSpeedMs = NAN;
float gpsCourseDeg = NAN;
uint8_t gpsSat = 0;

float axG = NAN;
float ayG = NAN;
float azG = NAN;
float gxDps = NAN;
float gyDps = NAN;
float gzDps = NAN;
float accelTotalG = NAN;

float batteryV = NAN;

float groundPressureHpa = NAN;
bool baroBaselineSet = false;
float baroAltM = NAN;

bool gpsValid = false;
bool bmeValid = false;
bool mpuValid = false;
bool nrfValid = false;
bool baroValid = false;

bool baroBaselineSet = false;
bool radioDetected = false;

bool bmeDetected = false;
bool mpuDetected = false;
bool radioDetected = false;
bool gpsSerialStarted = false;

bool startCommandReceived = false;
unsigned long descentStartMs = 0;

// =====================================================
// FUNCTION DECLARATIONS
// =====================================================
void updateGPS();
void readSensors1Hz();
void updateBME680();
void updateMPU6050();
void updateBattery();
void updateBarometricAltitude();
void updateMissionMode();

void pre_launch();


void sendTelemetryCycle();
void fillCorePacket(TelemetryCorePacket &core);
void fillImuPacket(TelemetryImuPacket &imu);

uint16_t checksum16(const void *data, size_t len);
int16_t toInt16Scaled(float value, float scale);
uint16_t toUInt16Scaled(float value, float scale);
int32_t toInt32Scaled(double value, double scale);
float readBatteryVoltage();

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===== CanSat Flight Controller Boot =====");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // GPS UART
  Serial2.begin(9600, SERIAL_8N1, GPS_RXD2, GPS_TXD2);
  gpsSerialStarted= true;
  // BME680
  if (!bme.begin(0x77)) {
    Serial.println("BME680 not found at 0x77, trying 0x76...");

    if (!bme.begin(0x76)) {
      Serial.println("ERROR: BME680 not detected");
      while (1);
    }
  }
  bmeDetected = bme.begin():
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  Serial.println("BME680 detected");

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not detected");
    while (1);
  }
  mpuDetected = mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 detected");

#if ENABLE_BATTERY_ADC
  analogReadResolution(12);
  pinMode(BATTERY_ADC_PIN, INPUT);
#endif

  // nRF24 SPI
  SPI.begin(18, 19, 23, NRF_CSN);
  radioDetected = radio.begin()

  if (!radio.begin()) {
    Serial.println("ERROR: nRF24 not detected");
    while (1);
  }
  radioDetected = radio.begin();
  radio.openWritingPipe(address);
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(32);
  radio.setAutoAck(true);
  radio.stopListening();

  Serial.println("nRF24 ready");
  Serial.println("Payload ready: GPS + BME680 + MPU6050 + nRF24 TX");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  // GPS must be decoded continuously, not only once per second.
  updateGPS();

  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;

    readSensors1Hz();
    updateMissionMode();
    sendTelemetryCycle();
  }
}

void pre_launch(){
  currentMode = MODE_PRE_LAUNCH;
  // pone el booleano gpsValid a true si detecta y mide el gps y falso si no
  gpsValid = gps.location.isValid() && gps.location.age() <3000 && gps.satellites.isValid() && gps.satellites.value() >0;
  //pone el booleano bmeValid a true si el bme mide correctamente y falso si no
  if (bmeDetected){
    bmeTempC = bme.temperature;
    pressureHpa = bme.pressure/100.0f;
    humidityPct = bme.humidity;
    gasKOhms = bme.gas_resisteance / 1000.0f:

    if(!baroBaselineSet && pressureHpa > 0){
      groundPressureHpa = pressureHpa;
      baroBaselineSet = true;
    }
  }else{
    bmeValid = false;
  }

  if(mpuDetected){
    sensors_event_t accelEvent;
    sensors_event_t gyroEvent;
    sensors_event_t tempEvent;

    mpu-getEvent(&accelEvent, &gyroEvent, &tempEvent);

    axG =accelEvent.acceleration.x / 9.80665f;
    ayG =accelEvent.acceleration.y / 9.80665f;
    azG =accelEvent.acceleration.z / 9.80665f;

    gxDps = gyroEvent.gyro.x * 57.2957795f;
    gyDps = gyroEvent.gyro.y * 57.2957795f;
    gzDps = gyroEvent.gyro.z * 57.2957795f;

    accelTotalG = sqrt(axG*axG+ayG*ayG+azG*azG);

        mpuValid =
      isfinite(axG) &&
      isfinite(ayG) &&
      isfinite(azG) &&
      isfinite(gxDps) &&
      isfinite(gyDps) &&
      isfinite(gzDps) &&
      accelTotalG > 0.2f &&
      accelTotalG < 8.0f;
  }else{
    mpuValid=false;
  }
  baroValid = bmeValid && baroBaselineSet && isfinite(pressureHpa) && pressureHpa>0;

  uint8_t statusFlags = 0;

  if (gpsValid) statusFlags |= FLAG_GPS_VALID;
  if (bmeValid) statusFlags |= FLAG_BME_VALID;
  if (mpuValid) statusFlags |= FLAG_MPU_VALID;
  if (varoValid) statusFlags |= FLAG_BARO_VALID;
  if (isfinite(batteryV)) statusFlags |= FLAG_BATTERY_VALID;

  bool readyForLaunch = bmeValid && mpuValid && baroValid && gpsValid && radioDetected;

  sendStatusPacket(statusFlags, readyForLaunch);

  bool commandStart = startCommandReceived;
  bool autoStart = descentConditionMet();

  if (readyForLaunch && (commandStart || autoStart)){
    currentMode = MODE_DESCENT;
    descentStartMs = millis();
  }


}
// =====================================================
// GPS UPDATE
// =====================================================
void updateGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

// =====================================================
// 1 Hz SENSOR CYCLE
// =====================================================
void readSensors1Hz() {
  gpsValid = gps.location.isValid();

  if (gpsValid) {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
  }

  gpsAltM = gps.altitude.isValid() ? gps.altitude.meters() : NAN;
  gpsSpeedMs = gps.speed.isValid() ? gps.speed.mps() : NAN;
  gpsCourseDeg = gps.course.isValid() ? gps.course.deg() : NAN;
  gpsSat = gps.satellites.isValid() ? gps.satellites.value() : 0;

  updateBME680();
  updateMPU6050();
  updateBattery();
  updateBarometricAltitude();

  Serial.println();
  Serial.println("===== 1 Hz SENSOR REPORT =====");

  Serial.print("Mode: ");
  Serial.println(currentMode);

  Serial.print("GPS valid: ");
  Serial.println(gpsValid ? "yes" : "no");

  Serial.print("Lat: ");
  Serial.println(gpsLat, 7);

  Serial.print("Lon: ");
  Serial.println(gpsLon, 7);

  Serial.print("GPS Alt m: ");
  Serial.println(gpsAltM);

  Serial.print("Baro Alt m: ");
  Serial.println(baroAltM);

  Serial.print("Temp C: ");
  Serial.println(bmeTempC);

  Serial.print("Pressure hPa: ");
  Serial.println(pressureHpa);

  Serial.print("Humidity %: ");
  Serial.println(humidityPct);

  Serial.print("Gas kOhms: ");
  Serial.println(gasKOhms);

  Serial.print("Accel G: ");
  Serial.print(axG);
  Serial.print(", ");
  Serial.print(ayG);
  Serial.print(", ");
  Serial.println(azG);

  Serial.print("Battery V: ");
  Serial.println(batteryV);
}

// =====================================================
// BME680
// =====================================================
void updateBME680() {
  if (bme.performReading()) {
    bmeValid = true;

    bmeTempC = bme.temperature;
    pressureHpa = bme.pressure / 100.0f;
    humidityPct = bme.humidity;
    gasKOhms = bme.gas_resistance / 1000.0f;

    if (!baroBaselineSet && pressureHpa > 0) {
      groundPressureHpa = pressureHpa;
      baroBaselineSet = true;

      Serial.print("Barometric baseline set: ");
      Serial.print(groundPressureHpa);
      Serial.println(" hPa");
    }
  } else {
    bmeValid = false;
    Serial.println("WARNING: BME680 reading failed");
  }
}

// =====================================================
// MPU6050
// =====================================================
void updateMPU6050() {
  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  mpuValid = true;

  // Adafruit MPU6050 acceleration is m/s^2.
  axG = accelEvent.acceleration.x / 9.80665f;
  ayG = accelEvent.acceleration.y / 9.80665f;
  azG = accelEvent.acceleration.z / 9.80665f;

  accelTotalG = sqrtf(axG * axG + ayG * ayG + azG * azG);

  // Gyro is rad/s. Convert to deg/s.
  gxDps = gyroEvent.gyro.x * 57.2957795f;
  gyDps = gyroEvent.gyro.y * 57.2957795f;
  gzDps = gyroEvent.gyro.z * 57.2957795f;
}

// =====================================================
// BATTERY
// =====================================================
void updateBattery() {
#if ENABLE_BATTERY_ADC
  batteryV = readBatteryVoltage();
#else
  batteryV = NAN;
#endif
}

float readBatteryVoltage() {
#if ENABLE_BATTERY_ADC
  int raw = analogRead(BATTERY_ADC_PIN);
  float adcVoltage = (raw / ADC_MAX) * ADC_REF;
  float dividerRatio = (BAT_R1 + BAT_R2) / BAT_R2;
  return adcVoltage * dividerRatio;
#else
  return NAN;
#endif
}

// =====================================================
// BAROMETRIC ALTITUDE
// Relative altitude using pressure baseline at boot.
// =====================================================
void updateBarometricAltitude() {
  if (!baroBaselineSet || !isfinite(pressureHpa) || pressureHpa <= 0) {
    baroAltM = NAN;
    return;
  }

  baroAltM = 44330.0f * (1.0f - powf(pressureHpa / groundPressureHpa, 0.1903f));
}

// =====================================================
// BASIC MISSION MODE LOGIC
// This is a simple placeholder. Later it can be replaced
// by telecommands and better launch/descent detection.
// =====================================================
void updateMissionMode() {
  static unsigned long descentStartMs = 0;
  static float previousBaroAlt = NAN;
  static unsigned long previousAltMs = 0;

  unsigned long now = millis();

  float verticalSpeed = NAN;

  if (isfinite(previousBaroAlt) && isfinite(baroAltM)) {
    float dt = (now - previousAltMs) / 1000.0f;

    if (dt > 0) {
      verticalSpeed = (baroAltM - previousBaroAlt) / dt;
    }
  }

  previousBaroAlt = baroAltM;
  previousAltMs = now;

  // Basic automatic transition:
  // If relative altitude becomes significant, consider mission active/descent.
  if (currentMode == MODE_PRE_LAUNCH) {
    if (isfinite(baroAltM) && fabs(baroAltM) > 5.0f) {
      currentMode = MODE_DESCENT;
      descentStartMs = now;
    }
  }

  // Basic post-impact detection:
  // after at least 10 s in descent, near ground altitude and low vertical speed.
  if (currentMode == MODE_DESCENT) {
    bool nearGround = isfinite(baroAltM) && fabs(baroAltM) < 3.0f;
    bool slowVertical = isfinite(verticalSpeed) && fabs(verticalSpeed) < 0.5f;
    bool enoughTime = (now - descentStartMs) > 10000;

    if (nearGround && slowVertical && enoughTime) {
      currentMode = MODE_POST_IMPACT;
    }
  }
}

// =====================================================
// SEND TWO 32-BYTE PACKETS
// =====================================================
void sendTelemetryCycle() {
  packetCounter++;

  TelemetryCorePacket core;
  TelemetryImuPacket imu;

  fillCorePacket(core);
  fillImuPacket(imu);

  radio.stopListening();

  bool coreOk = radio.write(&core, sizeof(core));
  delay(5);

  bool imuOk = radio.write(&imu, sizeof(imu));
  delay(5);

  radio.startListening();
  radio.stopListening();

  Serial.print("TX counter=");
  Serial.print(packetCounter);
  Serial.print(" CORE=");
  Serial.print(coreOk ? "OK" : "FAIL");
  Serial.print(" IMU=");
  Serial.println(imuOk ? "OK" : "FAIL");
}

// =====================================================
// FILL CORE PACKET
// =====================================================
void fillCorePacket(TelemetryCorePacket &core) {
  core.header.counter = packetCounter;
  core.header.type = PACKET_CORE;
  core.header.version = PACKET_VERSION;
  core.header.time_ms = millis();

  core.lat_e7 = gpsValid ? toInt32Scaled(gpsLat, 10000000.0) : 0;
  core.lon_e7 = gpsValid ? toInt32Scaled(gpsLon, 10000000.0) : 0;

  core.gps_alt_dm = toInt16Scaled(gpsAltM, 10.0f);
  core.baro_alt_dm = toInt16Scaled(baroAltM, 10.0f);

  core.speed_cms = toInt16Scaled(gpsSpeedMs, 100.0f);
  core.temp_centi = toInt16Scaled(bmeTempC, 100.0f);

  core.pressure_x10 = toUInt16Scaled(pressureHpa, 10.0f);
  core.humidity_centi = toUInt16Scaled(humidityPct, 100.0f);

  core.sat = gpsSat;
  core.mode = (uint8_t)currentMode;

  core.checksum = checksum16(&core, sizeof(core));
}

// =====================================================
// FILL IMU PACKET
// =====================================================
void fillImuPacket(TelemetryImuPacket &imu) {
  imu.header.counter = packetCounter;
  imu.header.type = PACKET_IMU;
  imu.header.version = PACKET_VERSION;
  imu.header.time_ms = millis();

  imu.ax_mg = toInt16Scaled(axG, 1000.0f);
  imu.ay_mg = toInt16Scaled(ayG, 1000.0f);
  imu.az_mg = toInt16Scaled(azG, 1000.0f);

  imu.gx_dps10 = toInt16Scaled(gxDps, 10.0f);
  imu.gy_dps10 = toInt16Scaled(gyDps, 10.0f);
  imu.gz_dps10 = toInt16Scaled(gzDps, 10.0f);

  imu.accel_total_mg = toUInt16Scaled(accelTotalG, 1000.0f);
  imu.battery_mv = toUInt16Scaled(batteryV, 1000.0f);
  imu.gas_x10 = toUInt16Scaled(gasKOhms, 10.0f);
  imu.course_deg10 = toInt16Scaled(gpsCourseDeg, 10.0f);

  uint8_t flags = 0;

  if (gpsValid) flags |= FLAG_GPS_VALID;
  if (bmeValid) flags |= FLAG_BME_VALID;
  if (mpuValid) flags |= FLAG_MPU_VALID;
  if (baroBaselineSet && isfinite(baroAltM)) flags |= FLAG_BARO_VALID;
  if (isfinite(batteryV)) flags |= FLAG_BATTERY_VALID;

  imu.flags = flags;
  imu.mode = (uint8_t)currentMode;

  imu.checksum = checksum16(&imu, sizeof(imu));
}

// =====================================================
// CHECKSUM
// Simple 16-bit additive checksum over all bytes except
// final checksum field.
// =====================================================
uint16_t checksum16(const void *data, size_t len) {
  const uint8_t *bytes = (const uint8_t *)data;
  uint16_t sum = 0;

  // Exclude last two bytes, where checksum is stored.
  for (size_t i = 0; i < len - 2; i++) {
    sum += bytes[i];
  }

  return sum;
}

// =====================================================
// SCALING HELPERS
// =====================================================
int16_t toInt16Scaled(float value, float scale) {
  if (!isfinite(value)) return INT16_MIN;

  long scaled = lroundf(value * scale);

  if (scaled > INT16_MAX) return INT16_MAX;
  if (scaled < INT16_MIN) return INT16_MIN;

  return (int16_t)scaled;
}

uint16_t toUInt16Scaled(float value, float scale) {
  if (!isfinite(value) || value < 0) return 0;

  unsigned long scaled = lroundf(value * scale);

  if (scaled > UINT16_MAX) return UINT16_MAX;

  return (uint16_t)scaled;
}

int32_t toInt32Scaled(double value, double scale) {
  if (!isfinite(value)) return 0;

  double scaled = value * scale;

  if (scaled > INT32_MAX) return INT32_MAX;
  if (scaled < INT32_MIN) return INT32_MIN;

  return (int32_t)llround(scaled);
}