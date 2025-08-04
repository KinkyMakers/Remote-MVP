# ESP-NOW Telemetry Receiver Guide

This guide explains how to set up an ESP32-S3 as an ESP-NOW receiver for battery test telemetry. The receiver will listen for ESP-NOW packets from your remote, parse the data, and forward it to Teleplot (via serial or UDP).

---

## 1. **Project Setup**
- Create a new PlatformIO project (or Arduino project) for ESP32-S3.
- Board: `esp32-s3-devkitc-1` (or your ESP32-S3 variant)
- Framework: Arduino

## 2. **Dependencies**
- ESP-NOW (built-in to ESP32 Arduino)
- (Optional) WiFi (for UDP forwarding)

## 3. **Receiver Code Outline**
- Initialize ESP-NOW in station mode.
- Register a receive callback.
- Parse incoming packets (expecting the same format as the sender: e.g., CSV or Teleplot lines).
- Forward each received line to:
  - **Serial** (for Teleplot via USB)
  - **OR UDP** (for Teleplot over WiFi)

## 4. **Example Code Snippet**
```cpp
#include <WiFi.h>
#include <esp_now.h>

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // Forward raw data to serial (Teleplot expects lines ending with \n)
  Serial.write(data, len);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1);
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESP-NOW receiver ready");
}

void loop() {
  // Nothing needed here
}
```

## 5. **Forwarding to Teleplot**
- **Serial:**
  - Connect the receiver to your computer via USB.
  - In Teleplot, open the receiver's serial port at 115200 baud.
- **UDP (optional):**
  - Add WiFi credentials and use WiFiUDP to send received lines to your computer's IP and port 47269.

## 6. **Testing**
- Flash this code to your receiver ESP32-S3.
- Power both sender and receiver.
- You should see telemetry appear in Teleplot.

---

**Tip:**
- Make sure the sender and receiver are on the same WiFi channel (default is usually fine if both are in station mode).
- If you want to parse/validate the data, you can add logic in `onDataRecv`.

---

**This file is a setup guide for the ESP-NOW receiver.** 