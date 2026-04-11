#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>

// Configuración GPS (Serial2)
#define RXD2 16
#define TXD2 17

// Configuración DS18B20
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

Adafruit_BME680 bme;
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

unsigned long tiempoAnterior = 0;
const long intervalo = 2000;  // cada 2 segundos imprimimos

// Variables para ángulos (filtro complementario)
float pitch = 0, roll = 0, yaw = 0;
unsigned long lastTimeMPU = 0;
const float alpha = 0.96;

// Offsets de calibración
float accBiasX = 0, accBiasY = 0, accBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

void calibrarMPU6050();
void getAccelAngles(float ax, float ay, float az, float &roll_rad, float &pitch_rad);
float rad2deg(float rad, bool to360 = false);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  sensors.begin();

  if (!bme.begin(0x77)) {
    Serial.println("No se encuentra el sensor BME680!");
  } else {
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }

  if (!mpu.begin()) {
    Serial.println("No se encuentra el sensor MPU6050!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 detectado. Calibrando...");
    calibrarMPU6050();
  }

  lastTimeMPU = micros();
  Serial.println("Iniciando lecturas (actualización continua de ángulos)");
}

void loop() {
  // --- Leer GPS constantemente ---
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  // --- LEER MPU6050 Y ACTUALIZAR ÁNGULOS EN CADA ITERACIÓN (rápido) ---
  sensors_event_t a, g, tempMpu;
  mpu.getEvent(&a, &g, &tempMpu);

  // Aplicar offsets de calibración
  float ax = a.acceleration.x - accBiasX;
  float ay = a.acceleration.y - accBiasY;
  float az = a.acceleration.z - accBiasZ;
  float gx = g.gyro.x - gyroBiasX;
  float gy = g.gyro.y - gyroBiasY;
  float gz = g.gyro.z - gyroBiasZ;

  // Delta tiempo para integración
  unsigned long now = micros();
  float dt = (now - lastTimeMPU) / 1000000.0;
  lastTimeMPU = now;

  // Calcular ángulos del acelerómetro (radianes)
  float roll_acc_rad, pitch_acc_rad;
  getAccelAngles(ax, ay, az, roll_acc_rad, pitch_acc_rad);

  // Velocidades angulares en rad/s
  float gyroRollRad = gx * PI / 180.0;
  float gyroPitchRad = gy * PI / 180.0;
  float gyroYawRad   = gz * PI / 180.0;

  // Filtro complementario para Roll y Pitch
  roll = alpha * (roll + gyroRollRad * dt) + (1 - alpha) * roll_acc_rad;
  pitch = alpha * (pitch + gyroPitchRad * dt) + (1 - alpha) * pitch_acc_rad;

  // Integración simple para Yaw (deriva)
  yaw += gyroYawRad * dt;

  // --- CADA 2 SEGUNDOS, IMPRIMIR TODOS LOS SENSORES ---
  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= intervalo) {
    tiempoAnterior = tiempoActual;

    // DS18B20
    sensors.requestTemperatures();
    float tempDS18B20 = sensors.getTempCByIndex(0);
    Serial.println("\n===== REPORTE DE SENSORES =====");
    Serial.print("TEMP (DS18B20): ");
    if (tempDS18B20 != DEVICE_DISCONNECTED_C) {
      Serial.print(tempDS18B20); Serial.println(" *C");
    } else {
      Serial.println("Error en sensor DS18B20");
    }

    // BME680
    if (bme.performReading()) {
      Serial.print("HUM: "); Serial.print(bme.humidity); Serial.print(" % | ");
      Serial.print("PRES: "); Serial.print(bme.pressure / 100.0); Serial.print(" hPa | ");
      Serial.print("GAS: "); Serial.print(bme.gas_resistance / 1000.0); Serial.println(" KOhms");
    } else {
      Serial.println("Error al leer BME680");
    }

    // Ángulos actuales (en grados, rango -180 a 180)
    float roll_deg = rad2deg(roll, false);
    float pitch_deg = rad2deg(pitch, false);
    float yaw_deg = rad2deg(yaw, false);
    Serial.print("ROLL: "); Serial.print(roll_deg); Serial.print("° | ");
    Serial.print("PITCH: "); Serial.print(pitch_deg); Serial.print("° | ");
    Serial.print("YAW: "); Serial.print(yaw_deg); Serial.println("° (deriva)");

    // GPS
    if (gps.location.isValid()) {
      Serial.print("LAT: "); Serial.print(gps.location.lat(), 6);
      Serial.print(" | LNG: "); Serial.print(gps.location.lng(), 6);
      Serial.print(" | ALT: "); Serial.print(gps.altitude.meters()); Serial.println(" m");
      Serial.print("SATS: "); Serial.print(gps.satellites.value());
      Serial.print(" | HORA: ");
      if (gps.time.hour() < 10) Serial.print("0");
      Serial.print(gps.time.hour() - 3);
      Serial.print(":");
      if (gps.time.minute() < 10) Serial.print("0");
      Serial.println(gps.time.minute());
    } else {
      Serial.println("GPS: Buscando satélites...");
    }
    Serial.println("===============================");
  }
}

// ---------- FUNCIONES DE CALIBRACIÓN Y ÁNGULOS ----------
void calibrarMPU6050() {
  const int numLecturas = 500;
  float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

  Serial.println("Mantén el MPU6050 completamente quieto. Calibrando...");
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
  accBiasZ = (sumAccZ / numLecturas) - 9.81;
  gyroBiasX = sumGyroX / numLecturas;
  gyroBiasY = sumGyroY / numLecturas;
  gyroBiasZ = sumGyroZ / numLecturas;

  Serial.println("Calibración completada. Offsets:");
  Serial.print("  Acc (X,Y,Z): "); Serial.print(accBiasX, 4); Serial.print(", "); Serial.print(accBiasY, 4); Serial.print(", "); Serial.println(accBiasZ, 4);
  Serial.print("  Gyro (X,Y,Z): "); Serial.print(gyroBiasX, 4); Serial.print(", "); Serial.print(gyroBiasY, 4); Serial.print(", "); Serial.println(gyroBiasZ, 4);
  delay(1000);
}

void getAccelAngles(float ax, float ay, float az, float &roll_rad, float &pitch_rad) {
  roll_rad = atan2(ay, az);
  pitch_rad = atan2(-ax, sqrt(ay*ay + az*az));
}

float rad2deg(float rad, bool to360) {
  float deg = rad * 180.0 / PI;
  if (to360) {
    deg = fmod(deg + 360.0, 360.0);
  }
  return deg;
}