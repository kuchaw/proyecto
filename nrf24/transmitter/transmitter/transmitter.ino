#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <TinyGPS++.h>

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

TinyGPSPlus gps;

// =========================
// BME680
// =========================
Adafruit_BME680 bme;

float lastBmeTemp = NAN;
float lastPressure = NAN;
float lastHumidity = NAN;
float lastGas = NAN;

// =========================
// Timing
// =========================
unsigned long lastSendMs = 0;
const unsigned long sendIntervalMs = 2000;

// =========================
// Function declarations
// =========================
void updateGPS();
void updateEnvironmentalSensors();
void sendRadioPacket();

void setup() {
  Serial.begin(115200);
  delay(1000);

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

    updateEnvironmentalSensors();
    sendRadioPacket();
  }
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
  if (!gps.location.isValid()) {
    Serial.println("GPS not valid yet, not sending nRF24 packet");
  }

  packet.counter++;

  packet.lat = gps.location.lat();
  packet.lon = gps.location.lng();
  packet.alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
  packet.sat = gps.satellites.isValid() ? gps.satellites.value() : 0;

  packet.temp = isnan(lastBmeTemp) ? -999.0f : lastBmeTemp;
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