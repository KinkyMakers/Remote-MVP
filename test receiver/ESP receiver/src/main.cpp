#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> // ESP-IDF WiFi header for channel setting

// Helper to print MAC address
void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; ++i) {
    if (i > 0) Serial.print(":");
    Serial.print(mac[i], HEX);
  }
}

// Callback when ESP-NOW data is received
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  Serial.print("[ESP-NOW] Packet received from ");
  printMac(mac);
  Serial.print(", length: ");
  Serial.println(len);

  Serial.print("[ESP-NOW] Data (hex): ");
  for (int i = 0; i < len && i < 16; ++i) { // Print up to 16 bytes
    if (i > 0) Serial.print(" ");
    Serial.printf("%02X", data[i]);
  }
  Serial.println();

  // Forward raw data to serial (Teleplot expects lines ending with \n)
  Serial.write(data, len);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // Station mode required for ESP-NOW

  // Force WiFi channel 1 before initializing ESP-NOW
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1);
  }
  Serial.println("ESP-NOW initialized successfully.");
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESP-NOW receiver ready");
}

void loop() {
  // Nothing needed here; all work is interrupt-driven
}