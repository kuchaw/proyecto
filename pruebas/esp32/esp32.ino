#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

#define ESPNOW_CHANNEL 6

struct __attribute__((packed)) TestPacket {
  uint32_t counter;
  uint32_t time_ms;
};


// ============================================
// Estadisticas
// ============================================

volatile uint32_t packetsReceived = 0;

volatile uint32_t firstCounter = 0;
volatile uint32_t lastCounter = 0;

volatile bool firstPacketReceived = false;

volatile int lastRSSI = 0;

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;


// ============================================
// Procesar paquete
// ============================================

void processPacket(
  const uint8_t *data,
  int len,
  int rssi
) {

  if (len != sizeof(TestPacket)) {
    return;
  }

  TestPacket incoming;

  memcpy(
    &incoming,
    data,
    sizeof(incoming)
  );

  portENTER_CRITICAL(&dataMux);

  if (!firstPacketReceived) {

    firstCounter = incoming.counter;
    lastCounter = incoming.counter;

    firstPacketReceived = true;

  } else {

    if (incoming.counter > lastCounter) {
      lastCounter = incoming.counter;
    }
  }

  packetsReceived++;
  lastRSSI = rssi;

  portEXIT_CRITICAL(&dataMux);
}


// ============================================
// Callback ESP-NOW
// ============================================

// Arduino ESP32 Core 3.x
#if ESP_ARDUINO_VERSION_MAJOR >= 3

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {

  int rssi = 0;

  if (info != nullptr &&
      info->rx_ctrl != nullptr) {

    rssi = info->rx_ctrl->rssi;
  }

  processPacket(
    data,
    len,
    rssi
  );
}

#else

// Arduino ESP32 Core 2.x
void OnDataRecv(
  const uint8_t *mac,
  const uint8_t *data,
  int len
) {

  // Core viejo no entrega RSSI directamente
  // en este callback.
  processPacket(
    data,
    len,
    0
  );
}

#endif


void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== ESP32 GROUND RANGE TEST ===");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);

  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  Serial.print("MAC receptor: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Canal: ");
  Serial.println(ESPNOW_CHANNEL);


  if (esp_now_init() != ESP_OK) {

    Serial.println(
      "ERROR inicializando ESP-NOW"
    );

    while (true) {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Esperando paquetes...");
}


// ============================================
// LOOP
// ============================================

void loop() {

  static uint32_t lastPrint = 0;

  if (millis() - lastPrint >= 1000) {

    lastPrint = millis();


    uint32_t received;
    uint32_t first;
    uint32_t last;
    int rssi;
    bool valid;


    portENTER_CRITICAL(&dataMux);

    received = packetsReceived;
    first = firstCounter;
    last = lastCounter;
    rssi = lastRSSI;
    valid = firstPacketReceived;

    portEXIT_CRITICAL(&dataMux);


    if (!valid) {

      Serial.println(
        "Sin paquetes..."
      );

      return;
    }


    // Cantidad de paquetes que deberiamos
    // haber recibido desde el primero.
    uint32_t expected =
      last - first + 1;


    uint32_t lost = 0;

    if (expected > received) {
      lost = expected - received;
    }


    float lossPercent = 0;

    if (expected > 0) {

      lossPercent =
        100.0f *
        ((float)lost /
         (float)expected);
    }


    Serial.println(
      "-----------------------------"
    );

    Serial.print("Ultimo paquete: ");
    Serial.println(last);

    Serial.print("Recibidos: ");
    Serial.println(received);

    Serial.print("Esperados: ");
    Serial.println(expected);

    Serial.print("Perdidos: ");
    Serial.println(lost);

    Serial.print("Packet loss: ");
    Serial.print(lossPercent, 2);
    Serial.println(" %");


#if ESP_ARDUINO_VERSION_MAJOR >= 3

    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");

#endif

  }
}