#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>

// =========================
// WiFi + server
// =========================
const char* ssid = "Fernandez";
const char* password = "16111505";
const char* serverUrl = "https://cansat1.onrender.com/api/telemetry";

// =========================
// GPS
// =========================
#define RXD2 16
#define TXD2 17
TinyGPSPlus gps;

// =========================
// DS18B20
// =========================
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// =========================
// BME680 + MPU6050
// =========================
Adafruit_BME680 bme;
Adafruit_MPU6050 mpu;

// =========================
// Timing
// =========================
unsigned long lastSendMs = 0;
const unsigned long sendIntervalMs = 2000;

// =========================
// MPU filter
// =========================
float pitch = 0.0f;
float roll  = 0.0f;
float yaw   = 0.0f;

unsigned long lastTimeMPU = 0;
const float alpha = 0.96f;

// Calibration offsets
float accBiasX = 0, accBiasY = 0, accBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Last sensor values
float lastTempDS18B20 = NAN;
float lastBmeTemp = NAN;
float lastHumidity = NAN;
float lastPressure = NAN;
float lastGas = NAN;

// =========================
// Function declarations
// =========================
void connectWiFi();
void calibrarMPU6050();
void getAccelAngles(float ax, float ay, float az, float &roll_rad, float &pitch_rad);
float rad2deg(float rad, bool to360 = false);
void updateGPS();
void updateMPU();
void updateEnvironmentalSensors();
void sendToServer();
String twoDigits(int value);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  ds18b20.begin();

  if (!bme.begin(0x77)) {
    Serial.println("No se encuentra el sensor BME680");
  } else {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    Serial.println("BME680 detectado");
  }

  if (!mpu.begin()) {
    Serial.println("No se encuentra el sensor MPU6050");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 detectado. Calibrando...");
    calibrarMPU6050();
  }

  connectWiFi();

  lastTimeMPU = micros();
  Serial.println("Sistema iniciado");
}

void loop() {
  updateGPS();
  updateMPU();

  unsigned long now = millis();
  if (now - lastSendMs >= sendIntervalMs) {
    lastSendMs = now;

    updateEnvironmentalSensors();
    sendToServer();
  }
}

void connectWiFi() {
  Serial.print("Conectando a WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void updateGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

void updateMPU() {
  sensors_event_t a, g, tempEvent;
  mpu.getEvent(&a, &g, &tempEvent);

  float ax = a.acceleration.x - accBiasX;
  float ay = a.acceleration.y - accBiasY;
  float az = a.acceleration.z - accBiasZ;
  float gx = g.gyro.x - gyroBiasX;
  float gy = g.gyro.y - gyroBiasY;
  float gz = g.gyro.z - gyroBiasZ;

  unsigned long now = micros();
  float dt = (now - lastTimeMPU) / 1000000.0f;
  lastTimeMPU = now;

  if (dt <= 0 || dt > 1.0f) {
    return;
  }

  float rollAccRad, pitchAccRad;
  getAccelAngles(ax, ay, az, rollAccRad, pitchAccRad);

  roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * rollAccRad;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitchAccRad;
  yaw   = yaw + gz * dt;
}

void updateEnvironmentalSensors() {
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  if (t != DEVICE_DISCONNECTED_C) {
    lastTempDS18B20 = t;
  }

  if (bme.performReading()) {
    lastBmeTemp   = bme.temperature;
    lastHumidity  = bme.humidity;
    lastPressure  = bme.pressure / 100.0f;      // hPa
    lastGas       = bme.gas_resistance / 1000.0f; // kOhms
  }

  Serial.println("\n===== REPORTE =====");

  if (!isnan(lastTempDS18B20)) {
    Serial.print("TEMP DS18B20: ");
    Serial.print(lastTempDS18B20);
    Serial.println(" C");
  } else {
    Serial.println("TEMP DS18B20: error");
  }

  if (!isnan(lastBmeTemp)) {
    Serial.print("TEMP BME680: ");
    Serial.print(lastBmeTemp);
    Serial.println(" C");
  }

  if (!isnan(lastHumidity)) {
    Serial.print("HUMEDAD: ");
    Serial.print(lastHumidity);
    Serial.println(" %");
  }

  if (!isnan(lastPressure)) {
    Serial.print("PRESION: ");
    Serial.print(lastPressure);
    Serial.println(" hPa");
  }

  if (!isnan(lastGas)) {
    Serial.print("GAS: ");
    Serial.print(lastGas);
    Serial.println(" kOhms");
  }

  Serial.print("ROLL: ");
  Serial.print(rad2deg(roll));
  Serial.print(" | PITCH: ");
  Serial.print(rad2deg(pitch));
  Serial.print(" | YAW: ");
  Serial.println(rad2deg(yaw));

  if (gps.location.isValid()) {
    Serial.print("LAT: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | LON: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print(" | ALT: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");
  } else {
    Serial.println("GPS: esperando fix");
  }

  Serial.println("===================");
}

void sendToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Reintentando...");
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return;
  }

  if (!gps.location.isValid()) {
    Serial.println("GPS no válido todavía, no se envía POST");
    return;
  }

  String dateStr = "0000-00-00";
  if (gps.date.isValid()) {
    dateStr =
      String(gps.date.year()) + "-" +
      twoDigits(gps.date.month()) + "-" +
      twoDigits(gps.date.day());
  }

  String timeStr = "00:00:00";
  if (gps.time.isValid()) {
    int hourUtc = gps.time.hour();
    int hourLocal = hourUtc - 3;
    if (hourLocal < 0) hourLocal += 24;

    timeStr =
      twoDigits(hourLocal) + ":" +
      twoDigits(gps.time.minute()) + ":" +
      twoDigits(gps.time.second());
  }

  float speedKmph = gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
  float courseDeg = gps.course.isValid() ? gps.course.deg() : 0.0f;
  float altitudeM = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  float rollDeg = rad2deg(roll);
  float pitchDeg = rad2deg(pitch);
  float yawDeg = rad2deg(yaw);

  // "temp" is the one your current HTML already uses
  // I assign it to DS18B20
  String json = "{";
  json += "\"lat\":" + String(gps.location.lat(), 6) + ",";
  json += "\"lon\":" + String(gps.location.lng(), 6) + ",";
  json += "\"alt\":" + String(altitudeM, 2) + ",";
  json += "\"sat\":" + String(sats) + ",";
  json += "\"speed\":" + String(speedKmph, 2) + ",";
  json += "\"course\":" + String(courseDeg, 2) + ",";

  json += "\"temp\":" + (isnan(lastTempDS18B20) ? String("null") : String(lastTempDS18B20, 2)) + ",";
  json += "\"temp_bme\":" + (isnan(lastBmeTemp) ? String("null") : String(lastBmeTemp, 2)) + ",";
  json += "\"pressure\":" + (isnan(lastPressure) ? String("null") : String(lastPressure, 2)) + ",";
  json += "\"humidity\":" + (isnan(lastHumidity) ? String("null") : String(lastHumidity, 2)) + ",";
  json += "\"gas\":" + (isnan(lastGas) ? String("null") : String(lastGas, 2)) + ",";

  json += "\"roll\":" + String(rollDeg, 2) + ",";
  json += "\"pitch\":" + String(pitchDeg, 2) + ",";
  json += "\"yaw\":" + String(yawDeg, 2) + ",";

  json += "\"date\":\"" + dateStr + "\",";
  json += "\"time\":\"" + timeStr + "\"";
  json += "}";

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, serverUrl);
  http.addHeader("Content-Type", "application/json");

  Serial.println("Enviando JSON:");
  Serial.println(json);

  int httpCode = http.POST(json);

  Serial.print("HTTP Response: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    String response = http.getString();
    Serial.println("Respuesta servidor:");
    Serial.println(response);
  } else {
    Serial.print("Error HTTP: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

void calibrarMPU6050() {
  const int numLecturas = 500;
  float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

  Serial.println("Mantén el MPU6050 quieto. Calibrando...");
  for (int i = 0; i < numLecturas; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumAccX += a.acceleration.x;
    sumAccY += a.acceleration.y;
    sumAccZ += a.acceleration.z;

    sumGyroX += g.gyro.x;
    sumGyroY += g.gyro.y;
    sumGyroZ += g.gyro.z;

    delay(5);
  }

  accBiasX = sumAccX / numLecturas;
  accBiasY = sumAccY / numLecturas;
  accBiasZ = (sumAccZ / numLecturas) - 9.81f;

  gyroBiasX = sumGyroX / numLecturas;
  gyroBiasY = sumGyroY / numLecturas;
  gyroBiasZ = sumGyroZ / numLecturas;

  Serial.println("Calibración completada");
}

void getAccelAngles(float ax, float ay, float az, float &roll_rad, float &pitch_rad) {
  roll_rad = atan2(ay, az);
  pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));
}

float rad2deg(float rad, bool to360) {
  float deg = rad * 180.0f / PI;
  if (to360) {
    deg = fmod(deg + 360.0f, 360.0f);
  }
  return deg;
}

String twoDigits(int value) {
  if (value < 10) return "0" + String(value);
  return String(value);
}