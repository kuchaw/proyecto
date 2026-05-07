#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
//#include <Adafruit_MPU6050.h>

// =========================
// nRF24
// =========================
#define NRF_CE 25
#define NRF_CSN 26

RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "GAAY1";

// Max 32 bytes
struct TelemetryPacket {
  uint32_t counter;
  float lat;
  float lon;
  float alt;
  float temp;
  float pressure;
  float humidity;
  uint8_t sat;
};

TelemetryPacket packet;

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
//DallasTemperature ds18b20(&oneWire);

// =========================
// BME680 + MPU6050
// =========================
Adafruit_BME680 bme;
//Adafruit_MPU6050 mpu;

// =========================
// Timing
// =========================
unsigned long lastSendMs = 0;
const unsigned long sendIntervalMs = 2000;

// =========================
// MPU filter
// =========================
/*float pitch = 0.0f;
float roll  = 0.0f;
float yaw   = 0.0f;

unsigned long lastTimeMPU = 0;
const float alpha = 0.96f;

float accBiasX = 0, accBiasY = 0, accBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
*/
// Last sensor values
//float lastTempDS18B20 = NAN;
float lastBmeTemp = NAN;
float lastHumidity = NAN;
float lastPressure = NAN;
float lastGas = NAN;

/*void calibrarMPU6050();
void getAccelAngles(float ax, float ay, float az, float &roll_rad, float &pitch_rad);
float rad2deg(float rad, bool to360 = false);*/
void updateGPS();
//void updateMPU();
void updateEnvironmentalSensors();
void sendRadioPacket();

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(18, 19, 23, NRF_CSN);
  Wire.begin();
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //ds18b20.begin();

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

  /*if (!mpu.begin()) {
    Serial.println("No se encuentra el sensor MPU6050");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 detectado. Calibrando...");
    calibrarMPU6050();
  }*/


  if (!radio.begin()) {
    Serial.println("nRF24 no detectado");
    while (1);
  }

  radio.openWritingPipe(address);
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.stopListening();

  packet.counter = 0;
  //lastTimeMPU = micros();

  Serial.println("Payload listo: sensores + nRF24 TX");
}

void loop() {
  updateGPS();
  //updateMPU();

  unsigned long now = millis();
  if (now - lastSendMs >= sendIntervalMs) {
    lastSendMs = now;

    updateEnvironmentalSensors();
    sendRadioPacket();
  }
}

void updateGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

/*void updateMPU() {
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

  if (dt <= 0 || dt > 1.0f) return;

  float rollAccRad, pitchAccRad;
  getAccelAngles(ax, ay, az, rollAccRad, pitchAccRad);

  roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * rollAccRad;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitchAccRad;
  yaw   = yaw + gz * dt;
}*/

void updateEnvironmentalSensors() {
  //ds18b20.requestTemperatures();
  //float t = ds18b20.getTempCByIndex(0);

 /* if (t != DEVICE_DISCONNECTED_C) {
    lastTempDS18B20 = t;
  }*/

  if (bme.performReading()) {
    lastBmeTemp   = bme.temperature;
    lastHumidity  = bme.humidity;
    lastPressure  = bme.pressure / 100.0f;
    lastGas       = bme.gas_resistance / 1000.0f;
  }

  Serial.println("\n===== PAYLOAD SENSOR REPORT =====");

  Serial.print("GPS valid: ");
  Serial.println(gps.location.isValid() ? "yes" : "no");

  /*Serial.print("Temp DS18B20: ");
  Serial.println(lastTempDS18B20);
*/
  Serial.print("Pressure: ");
  Serial.println(lastPressure);

  Serial.print("Humidity: ");
  Serial.println(lastHumidity);

  /*Serial.print("ROLL: ");
  Serial.print(rad2deg(roll));
  Serial.print(" | PITCH: ");
  Serial.print(rad2deg(pitch));
  Serial.print(" | YAW: ");
  Serial.println(rad2deg(yaw));*/
}

void sendRadioPacket() {
  if (!gps.location.isValid()) {
    Serial.println("GPS no válido todavía, no se envía por nRF24");
    return;
  }

  packet.counter++;
  packet.lat = gps.location.lat();
  packet.lon = gps.location.lng();
  packet.alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
  packet.sat = gps.satellites.isValid() ? gps.satellites.value() : 0;

  //packet.temp = isnan(lastTempDS18B20) ? -999.0f : lastTempDS18B20;
  packet.pressure = isnan(lastPressure) ? -999.0f : lastPressure;
  packet.humidity = isnan(lastHumidity) ? -999.0f : lastHumidity;

  bool ok = radio.write(&packet, sizeof(packet));

  Serial.print("nRF24 packet ");
  Serial.print(packet.counter);
  Serial.print(" size=");
  Serial.print(sizeof(packet));
  Serial.print(" bytes status: ");
  Serial.println(ok ? "OK" : "FAIL");
}
/*
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
  }*//*

  accBiasX = sumAccX / numLecturas;
  accBiasY = sumAccY / numLecturas;
  accBiasZ = (sumAccZ / numLecturas) - 9.81f;

  gyroBiasX = sumGyroX / numLecturas;
  gyroBiasY = sumGyroY / numLecturas;
  gyroBiasZ = sumGyroZ / numLecturas;

  Serial.println("Calibración completada");
}*/

/*void getAccelAngles(float ax, float ay, float az, float &roll_rad, float &pitch_rad) {
  roll_rad = atan2(ay, az);
  pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));
}

float rad2deg(float rad, bool to360) {
  float deg = rad * 180.0f / PI;
  if (to360) {
    deg = fmod(deg + 360.0f, 360.0f);
  }
  return deg;
}*/