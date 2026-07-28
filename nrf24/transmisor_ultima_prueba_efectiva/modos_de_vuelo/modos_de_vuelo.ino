
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>

HardwareSerial CamSerial(1);

// =========================
// nRF24
// =========================
#define NRF_CE 25
#define NRF_CSN 26

RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "GAAY1";

// Keep packet explicitly 32 bytes for nRF24 compatibility
struct TelemetryPacket {
  uint32_t counter;
  float lat;
  float lon;
  float alt;
  float temp;
  float pressure;
  float humidity;
  uint8_t sat;
  uint8_t padding[3];
};

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
// MPU6050
// =========================
Adafruit_MPU6050 mpu;

float ax = NAN;
float ay = NAN;
float az = NAN;

float gx = NAN;
float gy = NAN;
float gz = NAN;

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
void sendRadioPacket();
void handlePrelaunch();
void handleDescent();
void handlePostImpact();
void updateMPU6050();
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

  // nRF24 SPI
  SPI.begin(18, 19, 23, NRF_CSN);

  if (!radio.begin()) {
    Serial.println("nRF24 not detected");
    while (1);
  }

  radio.openWritingPipe(address);
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.stopListening();

  packet.counter = 0;
  packet.padding[0] = 0;
  packet.padding[1] = 0;
  packet.padding[2] = 0;

  Serial.println("Payload ready: GPS + BME680 + nRF24 TX");
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

  sendRadioPacket();
}
void handleDescent() {
  Serial.println("\n===== DESCENT MODE =====");

  updateEnvironmentalSensors();

  sendRadioPacket();
}
void handlePostImpact() {
  Serial.println("\n===== POST IMPACT MODE =====");
  sendRadioPacket();
}

void updateGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
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

void sendRadioPacket() {

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

  packet.padding[1] = 1;  // current GPS valid
} 
else if (hasLastKnownGps) {
  packet.lat = lastKnownLat;
  packet.lon = lastKnownLon;
  packet.alt = lastKnownAlt;
  packet.sat = lastKnownSat;

  packet.padding[1] = 2;  // using last known GPS
} 
else {
  packet.lat = 0.0f;
  packet.lon = 0.0f;
  packet.alt = 0.0f;
  packet.sat = 0;

  packet.padding[1] = 0;  // no GPS available
}

  packet.temp = isnan(lastBmeTemp) ? -999.0f : lastBmeTemp;
  packet.pressure = isnan(lastPressure) ? -999.0f : lastPressure;
  packet.humidity = isnan(lastHumidity) ? -999.0f : lastHumidity;

  packet.padding[0] = (uint8_t)currentMode;
  packet.padding[1] = gpsOk ? 1 : 0;
  packet.padding[2] = 0;

  bool ok = radio.write(&packet, sizeof(packet));

  CamSerial.write((uint8_t *)&packet, sizeof(packet));

  Serial.print("nRF24 packet ");
  Serial.print(packet.counter);
  Serial.print(" size=");
  Serial.print(sizeof(packet));
  Serial.print(" bytes status: ");
  Serial.println(ok ? "OK" : "FAIL");
}
