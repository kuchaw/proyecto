#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 6
#define SEND_INTERVAL_MS 100

// Broadcast: todos los ESP32 con ESP-NOW en este canal pueden recibir.
uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

struct __attribute__((packed)) TestPacket {
  uint32_t counter;
  uint32_t time_ms;
};

TestPacket packet;

uint32_t lastSend = 0;
uint32_t counter = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== ESP32-CAM RANGE TEST ===");

  // Activamos solamente la radio WiFi del ESP32.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);

  // Desactivar ahorro de energía para una prueba de RF más consistente.
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Ambos dispositivos tienen que estar en el mismo canal.
  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  /*
   * Potencia máxima = 11 dBm.
   *
   * esp_wifi_set_max_tx_power usa unidades de 0.25 dBm:
   * 44 -> nivel de 11 dBm.
   *
   * Para la competencia todavía habrá que sumar la
   * ganancia de la antena para verificar EIRP.
   */
  esp_err_t powerResult = esp_wifi_set_max_tx_power(44);

  Serial.print("TX power config: ");
  Serial.println(powerResult == ESP_OK ? "OK" : "ERROR");

  Serial.print("MAC ESP32-CAM: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Canal: ");
  Serial.println(ESPNOW_CHANNEL);

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR inicializando ESP-NOW");
    while (true) {
      delay(1000);
    }
  }

  // Para enviar broadcast hay que registrar el broadcast como peer.
  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    broadcastAddress,
    6
  );

  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR agregando broadcast peer");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("ESP-NOW inicializado.");
  Serial.println("Transmitiendo...");
}

void loop() {

  if (millis() - lastSend >= SEND_INTERVAL_MS) {

    lastSend = millis();

    packet.counter = counter;
    packet.time_ms = millis();

    esp_err_t result = esp_now_send(
      broadcastAddress,
      (uint8_t *)&packet,
      sizeof(packet)
    );

    if (result != ESP_OK) {
      Serial.print("Error envio paquete ");
      Serial.println(counter);
    }

    // No llenamos demasiado el Serial.
    if (counter % 10 == 0) {
      Serial.print("TX #");
      Serial.print(counter);
      Serial.print("   t=");
      Serial.print(packet.time_ms);
      Serial.println(" ms");
    }

    counter++;
  }
}