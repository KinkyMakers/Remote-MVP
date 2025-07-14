#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <FastLED.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "LSM6DS3_Simple.h"

// Pin definitions (from Board Config.md)
#define TFT_CS   4   // Screen Cable Select
#define TFT_DC   7   // Screen A0
#define TFT_RST  16  // Screen Reset
#define TFT_SCLK 5   // Screen Clock
#define TFT_MOSI 6   // Screen Data
#define TFT_BL   15  // Screen Backlight

#define LED_PIN      37  // WS2812B LED pin
#define NUM_LEDS     3   // Number of LEDs
#define BUZZER_PIN   2   // Buzzer
#define VIBRATOR_PIN 47  // Vibrator

// I2C pins for battery fuel gauge
#define I2C_SDA      8   // I2C SDA
#define I2C_SCL      9   // I2C SCL

// Button pins from Board Config.md
#define BTN_R_SHOULDER 1   // Right Shoulder Button
#define BTN_L_SHOULDER 48  // Left Shoulder Button
#define BTN_L_ENC_A   36   // Left Encoder A
#define BTN_L_ENC_B   35   // Left Encoder B
#define BTN_R_ENC_A   42   // Right Encoder A
#define BTN_R_ENC_B   41   // Right Encoder B
#define BTN_UNDER_L   38   // Under Screen Left Button
#define BTN_UNDER_C   39   // Under Screen Centre Button
#define BTN_UNDER_R   40   // Under Screen Right Button

// Encoder pins (from Board Config.md)
#define ENC_L_A 36
#define ENC_L_B 35
#define ENC_R_A 42
#define ENC_R_B 41

// MAX17048 Fuel Gauge I2C Address
#define MAX17048_ADDR 0x36

// MAX17048 Register addresses
#define MAX17048_VCELL_REG    0x02
#define MAX17048_SOC_REG      0x04
#define MAX17048_MODE_REG     0x06
#define MAX17048_VERSION_REG   0x08
#define MAX17048_CONFIG_REG    0x0C
#define MAX17048_COMMAND_REG   0xFE

// Battery fuel gauge functions (FIXED VERSION)
void resetMAX17048(); // Forward declaration

bool checkBatteryGauge() {
  Wire.beginTransmission(MAX17048_ADDR);
  byte error = Wire.endTransmission();
  return (error == 0);
}

void scanI2CDevices() {
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int deviceCount = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      // Print known device names
      switch (address) {
        case 0x36:
          Serial.println("  -> MAX17048 (Battery Monitor)");
          break;
        case 0x6A:
          Serial.println("  -> LSM6DS3 (Accelerometer)");
          break;
        default:
          Serial.println("  -> Unknown device");
          break;
      }
      deviceCount++;
    }
  }
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }
}

// Fixed battery initialization
bool initializeMAX17048() {
  uint8_t addr = 0x36;
  
  // Step 1: Check if device responds
  Wire.beginTransmission(addr);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.printf("Device not responding at 0x%02X\n", addr);
    return false;
  }
  
  // Step 2: Read version register
  Wire.beginTransmission(addr);
  Wire.write(MAX17048_VERSION_REG);
  error = Wire.endTransmission(false);
  
  if (error == 0) {
    Wire.requestFrom((int)addr, 1);
    if (Wire.available() >= 1) {
      uint8_t version = Wire.read();
      Serial.printf("Version register: 0x%02X\n", version);
      
      // If version is 0x00, the device needs initialization
      if (version == 0x00) {
        Serial.println("Device appears to be in reset state, initializing...");
        resetMAX17048();
        delay(1000); // Wait for reset to complete
        
        // Read version again
        Wire.beginTransmission(addr);
        Wire.write(MAX17048_VERSION_REG);
        error = Wire.endTransmission(false);
        
        if (error == 0) {
          Wire.requestFrom((int)addr, 1);
          if (Wire.available() >= 1) {
            version = Wire.read();
            Serial.printf("Version after reset: 0x%02X\n", version);
          }
        }
      }
    }
  }
  
  return true;
}

void resetMAX17048() {
  uint8_t addr = 0x36;
  
  Serial.println("Sending reset command to MAX17048...");
  
  // Send reset command
  Wire.beginTransmission(addr);
  Wire.write(MAX17048_COMMAND_REG);
  Wire.write(0x00);  // Reset command
  Wire.write(0x54);  // Reset command
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("✓ Reset command sent successfully");
  } else {
    Serial.println("✗ Failed to send reset command");
  }
}

// Fixed battery reading functions
float readBatteryVoltage() {
  uint8_t addr = 0x36;
  
  Wire.beginTransmission(addr);
  Wire.write(MAX17048_VCELL_REG);
  byte error = Wire.endTransmission(false);
  
  if (error != 0) return 0.0;
  
  Wire.requestFrom((int)addr, 2);
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    
    // VCELL is 12-bit, stored in bits 15-4
    uint16_t raw = ((msb << 4) | (lsb >> 4));
    
    // Correct conversion: 1.25mV per LSB
    float voltage = raw * 0.00125;
    
    return voltage;
  }
  return 0.0;
}

int readBatteryPercent() {
  uint8_t addr = 0x36;
  
  Wire.beginTransmission(addr);
  Wire.write(MAX17048_SOC_REG);
  byte error = Wire.endTransmission(false);
  
  if (error != 0) return 0;
  
  Wire.requestFrom((int)addr, 2);
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    
    // SOC is 16-bit, stored as 1/256% resolution
    uint16_t raw = ((msb << 8) | lsb);
    int percent = raw / 256;
    
    // Clamp to 0-100 range
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    
    return percent;
  }
  return 0;
}

// Other available pins from Board Config.md (unassigned here):
// 8, 9, 13, 14, 17, 18, 19, 20, 21

// Hardware objects
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
CRGB leds[NUM_LEDS];
LSM6DS3_Simple imu;

// Shared variables with protection
volatile int leftEncoderValue = 128; // 0-255 for brightness
volatile int rightEncoderValue = 0;  // 0-255 for hue
volatile bool dashboardReady = false;

volatile int buttonStates = 0; // For button status
volatile float imuX = 0, imuY = 0, imuZ = 0; // For IMU data
volatile float batteryVoltage = 0.0;
volatile int batteryPercent = 0;
volatile bool batteryGaugeFound = false;
SemaphoreHandle_t encoderMutex = NULL;

// Sensor fusion variables
volatile float fusedRoll = 0.0;
volatile float fusedPitch = 0.0;
volatile float fusedYaw = 0.0;
volatile unsigned long lastFusionUpdate = 0;

// Buzzer and vibrator pattern state
volatile bool buzzerActive = false;
volatile unsigned long buzzerStart = 0;
volatile int buzzerStep = 0;

volatile bool vibeActive = false;
volatile unsigned long vibeStart = 0;
volatile int vibeStep = 0;

// Mario Bros theme state
volatile bool marioPlaying = false;
volatile int marioNoteIndex = 0;
volatile unsigned long lastNoteTime = 0;

// Mario Bros theme (iconic opening phrase)
const int marioNotes[] = {
  659, 659, 0, 659, 0, 523, 659, 784, 0, 392
};
const int marioDurations[] = {
  125, 125, 125, 125, 125, 125, 125, 250, 125, 250
};
const int marioLength = sizeof(marioNotes) / sizeof(marioNotes[0]);

// Vibe pattern (on/off in ms)
const int vibePattern[] = { 150, 100, 150, 100, 300, 0 }; // ms (on, off, on, off, on, end)
const int vibeLength = 5;

// Mario coin sound (freq, duration in ms)
const int marioCoinNotes[] = {1319, 1568, 2637};
const int marioCoinDurations[] = {60, 60, 120};
const int marioCoinLength = 3;

void playMarioCoin() {
  for (int i = 0; i < marioCoinLength; ++i) {
    tone(BUZZER_PIN, marioCoinNotes[i]);
    delay(marioCoinDurations[i]);
    noTone(BUZZER_PIN);
    delay(20);
  }
}

// Task handles
TaskHandle_t dashboardTaskHandle = NULL;

// IMU functions
void updateSensorFusion(float ax, float ay, float az, float gx, float gy, float gz);

bool initIMU() {
  Serial.println("Initializing IMU (LSM6DS3_Simple)...");
  delay(100);
  if (!imu.begin(Wire, 0x6A)) {
    Serial.println("IMU not found!");
    return false;
  }
  Serial.println("IMU initialized successfully!");
  return true;
}

void readIMU() {
  float x, y, z;
  imu.readAccel(x, y, z);
  imuX = x;
  imuY = y;
  imuZ = z;
  float gx, gy, gz;
  imu.readGyro(gx, gy, gz);
  
  // Update sensor fusion
  updateSensorFusion(x, y, z, gx, gy, gz);
  
  // Debug IMU readings (only first few times)
  static int imuDebugCount = 0;
  if (imuDebugCount < 10) {
    Serial.printf("IMU Debug: Accel X=%.3f, Y=%.3f, Z=%.3f\n", imuX, imuY, imuZ);
    Serial.printf("IMU Debug: Gyro X=%.3f, Y=%.3f, Z=%.3f\n", gx, gy, gz);
    imu.debugRaw();
    imuDebugCount++;
  }
}

// Complementary filter for sensor fusion
void updateSensorFusion(float ax, float ay, float az, float gx, float gy, float gz) {
  unsigned long now = millis();
  float dt = (now - lastFusionUpdate) / 1000.0f; // Convert to seconds
  if (dt > 0.1f) dt = 0.1f; // Limit dt to prevent instability
  lastFusionUpdate = now;
  
  // Complementary filter coefficients
  float alpha = 0.98f; // Gyro trust factor
  float beta = 1.0f - alpha; // Accel trust factor
  
  // Calculate roll and pitch from accelerometer
  float accelRoll = atan2(ay, az) * 180.0f / PI;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  
  // Integrate gyroscope data
  fusedRoll = alpha * (fusedRoll + gx * dt) + beta * accelRoll;
  fusedPitch = alpha * (fusedPitch + gy * dt) + beta * accelPitch;
  fusedYaw = fusedYaw + gz * dt; // Yaw has no accel reference, so just integrate
  
  // Keep yaw in reasonable range
  if (fusedYaw > 180.0f) fusedYaw -= 360.0f;
  if (fusedYaw < -180.0f) fusedYaw += 360.0f;
}

// --- Fixed encoder ISRs with correct direction ---
void IRAM_ATTR handleLeftEncoderA() {
  static bool lastA = LOW, lastB = LOW;
  bool a = digitalRead(ENC_L_A);
  bool b = digitalRead(ENC_L_B);
  
  if (a != lastA) {
    if (a == HIGH) {
      if (b == HIGH) { // Swapped A/B logic
        leftEncoderValue = (leftEncoderValue + 8) % 256; // Wrap around
      } else {
        leftEncoderValue = (leftEncoderValue - 8 + 256) % 256; // Wrap around
      }
    }
    lastA = a;
    lastB = b;
  }
}

void IRAM_ATTR handleRightEncoderA() {
  static bool lastA = LOW, lastB = LOW;
  bool a = digitalRead(ENC_R_A);
  bool b = digitalRead(ENC_R_B);
  
  if (a != lastA) {
    if (a == HIGH) {
      if (b == HIGH) { // Swapped A/B logic
        rightEncoderValue = (rightEncoderValue + 8) % 256; // Wrap around
      } else {
        rightEncoderValue = (rightEncoderValue - 8 + 256) % 256; // Wrap around
      }
    }
    lastA = a;
    lastB = b;
  }
}

void IRAM_ATTR leftButtonISR() {
  if (!marioPlaying) {
    marioPlaying = true;
    marioNoteIndex = 0;
    lastNoteTime = millis();
    // Start playing the first note
    if (marioNotes[0] > 0) {
      tone(BUZZER_PIN, marioNotes[0]);
    }
  }
}

void IRAM_ATTR rightButtonISR() {
  Serial.println("Right button pressed!");
  vibeActive = true;
  vibeStart = millis();
  vibeStep = 0;
}

// --- Full dashboard update task for core 1 ---
void dashboardTask(void *param) {
  (void)param;
  
  // Wait for initialization to complete
  while (!dashboardReady) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  // Cache previous values to avoid unnecessary redraws
  static int lastBrightness = -1;
  static int lastHue = -1;
  static int lastBatteryPercent = -1;
  static float lastBatteryVoltage = -1;
  static int lastLShoulder = -1;
  static int lastRShoulder = -1;
  static int lastUnderL = -1;
  static int lastUnderC = -1;
  static int lastUnderR = -1;
  static int lastLEncA = -1;
  static int lastLEncB = -1;
  static int lastREncA = -1;
  static int lastREncB = -1;
  static float lastGx = -999, lastGy = -999, lastGz = -999;
  static float lastAx = -999, lastAy = -999, lastAz = -999;
  
  while (true) {
    // Get encoder values safely
    int brightness, hue;
    if (encoderMutex != NULL && xSemaphoreTake(encoderMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      brightness = leftEncoderValue;
      hue = rightEncoderValue;
      xSemaphoreGive(encoderMutex);
    } else {
      brightness = 128;
      hue = 0;
    }
    
    // Read battery status (every few cycles to avoid slowing down display)
    static int batteryCounter = 0;
    if (++batteryCounter >= 20) { // Read battery every 20 cycles (~1 second)
      batteryVoltage = readBatteryVoltage();
      batteryPercent = readBatteryPercent();
      batteryCounter = 0;
    }
    
    // Read IMU data
    readIMU();
    
    // Update LEDs
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(hue, 255, brightness);
    }
    FastLED.show();
    
    // Update full dashboard (optimized - only redraw when values change)
    tft.setTextSize(2); // 40% bigger text
    
    // Get current button states
    int lShoulder = !digitalRead(BTN_L_SHOULDER);
    int rShoulder = !digitalRead(BTN_R_SHOULDER);
    int underL = !digitalRead(BTN_UNDER_L);
    int underC = !digitalRead(BTN_UNDER_C);
    int underR = !digitalRead(BTN_UNDER_R);
    int lEncA = digitalRead(ENC_L_A);
    int lEncB = digitalRead(ENC_L_B);
    int rEncA = digitalRead(ENC_R_A);
    int rEncB = digitalRead(ENC_R_B);
    
    // Line 1: Shoulder Buttons (only update if changed)
    if (lShoulder != lastLShoulder || rShoulder != lastRShoulder) {
      tft.fillRect(0, 8, 240, 16, ST77XX_BLACK);
      tft.setCursor(0, 8);
      tft.setTextColor(ST77XX_WHITE); // Always start line white
      tft.setTextColor(lShoulder ? ST77XX_GREEN : ST77XX_WHITE);
      tft.print("L.Shoulder");
      tft.setTextColor(ST77XX_WHITE);
      tft.print(" ");
      tft.setTextColor(rShoulder ? ST77XX_GREEN : ST77XX_WHITE);
      tft.print("R.Shoulder");
      tft.setTextColor(ST77XX_WHITE); // Reset to white for next line
      lastLShoulder = lShoulder;
      lastRShoulder = rShoulder;
    }
    
    // Line 2: Under Screen Buttons (only update if changed)
    if (underL != lastUnderL || underC != lastUnderC || underR != lastUnderR) {
      tft.fillRect(0, 32, 240, 16, ST77XX_BLACK);
      tft.setCursor(0, 32);
      tft.setTextColor(ST77XX_WHITE); // Always start line white
      tft.setTextColor(underL ? ST77XX_GREEN : ST77XX_WHITE);
      tft.print("L");
      tft.setTextColor(ST77XX_WHITE);
      tft.print(" ");
      tft.setTextColor(underC ? ST77XX_GREEN : ST77XX_WHITE);
      tft.print("C");
      tft.setTextColor(ST77XX_WHITE);
      tft.print(" ");
      tft.setTextColor(underR ? ST77XX_GREEN : ST77XX_WHITE);
      tft.print("R");
      tft.setTextColor(ST77XX_WHITE); // Reset to white for next line
      lastUnderL = underL;
      lastUnderC = underC;
      lastUnderR = underR;
    }
    
    // Line 3: Encoder States (only update if changed)
    if (lEncA != lastLEncA || lEncB != lastLEncB || rEncA != lastREncA || rEncB != lastREncB) {
      tft.fillRect(0, 56, 240, 16, ST77XX_BLACK);
      tft.setCursor(0, 56);
      tft.setTextColor(ST77XX_WHITE); // Always start line white
      tft.print("L:");
      tft.setTextColor(lEncA ? ST77XX_GREEN : ST77XX_RED);
      tft.print(lEncA);
      tft.setTextColor(ST77XX_WHITE);
      tft.print(" ");
      tft.setTextColor(lEncB ? ST77XX_GREEN : ST77XX_RED);
      tft.print(lEncB);
      tft.setTextColor(ST77XX_WHITE);
      tft.print(" R:");
      tft.setTextColor(rEncA ? ST77XX_GREEN : ST77XX_RED);
      tft.print(rEncA);
      tft.setTextColor(ST77XX_WHITE);
      tft.print(" ");
      tft.setTextColor(rEncB ? ST77XX_GREEN : ST77XX_RED);
      tft.print(rEncB);
      tft.setTextColor(ST77XX_WHITE); // Reset to white for next line
      lastLEncA = lEncA;
      lastLEncB = lEncB;
      lastREncA = rEncA;
      lastREncB = rEncB;
    }
    
    // Line 4: Encoder Values (only update if changed)
    if (brightness != lastBrightness || hue != lastHue) {
      tft.fillRect(0, 80, 240, 16, ST77XX_BLACK);
      tft.setCursor(0, 80);
      tft.setTextColor(ST77XX_WHITE); // Always start line white
      tft.print("L:");
      tft.print(brightness);
      tft.print(" R:");
      tft.print(hue);
      lastBrightness = brightness;
      lastHue = hue;
    }
    
    // Line 5: Battery Status (only update if changed)
    if (batteryPercent != lastBatteryPercent || abs(batteryVoltage - lastBatteryVoltage) > 0.01) {
      tft.fillRect(0, 104, 240, 16, ST77XX_BLACK);
      tft.setCursor(0, 104);
      tft.setTextColor(ST77XX_WHITE); // Always start line white
      tft.print("Batt:");
      tft.print(batteryPercent);
      tft.print("% ");
      tft.print(batteryVoltage, 2);
      tft.print("V");
      lastBatteryPercent = batteryPercent;
      lastBatteryVoltage = batteryVoltage;
    }
    
    // Line 6: IMU Data (only update if changed significantly)
    if (abs(imuX - lastAx) > 0.01 || abs(imuY - lastAy) > 0.01 || abs(imuZ - lastAz) > 0.01) {
      tft.fillRect(0, 128, 240, 16, ST77XX_BLACK);
      tft.setCursor(0, 128);
      tft.setTextColor(ST77XX_WHITE); // Always start line white
      tft.print("A:");
      tft.print(imuX, 2);
      tft.print(" ");
      tft.print(imuY, 2);
      tft.print(" ");
      tft.print(imuZ, 2);
      lastAx = imuX;
      lastAy = imuY;
      lastAz = imuZ;
    }
    
    // Line 7: Orientation (only update if changed significantly)
    if (abs(fusedRoll - lastGx) > 1.0 || abs(fusedPitch - lastGy) > 1.0 || abs(fusedYaw - lastGz) > 1.0) {
      tft.fillRect(0, 152, 240, 16, ST77XX_BLACK);
      tft.setCursor(0, 152);
      tft.setTextColor(ST77XX_WHITE); // Always start line white
      tft.print("O:");
      tft.print(fusedRoll, 0);
      tft.print(" ");
      tft.print(fusedPitch, 0);
      tft.print(" ");
      tft.print(fusedYaw, 0);
      lastGx = fusedRoll;
      lastGy = fusedPitch;
      lastGz = fusedYaw;
    }
    
    // Line 8: Status
    tft.fillRect(0, 176, 240, 16, ST77XX_BLACK);
    tft.setCursor(0, 176);
    tft.setTextColor(ST77XX_WHITE); // Always start line white
    tft.print("Status: ");
    if (batteryGaugeFound) {
      tft.setTextColor(ST77XX_GREEN);
      tft.print("OK");
    } else {
      tft.setTextColor(ST77XX_RED);
      tft.print("ERR");
    }
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" ");
    tft.print(millis() / 1000);
    tft.print("s");
    
    // Handle Mario theme
    if (marioPlaying) {
      unsigned long now = millis();
      if (now - lastNoteTime >= marioDurations[marioNoteIndex]) {
        marioNoteIndex++;
        if (marioNoteIndex >= marioLength) {
          marioPlaying = false;
          noTone(BUZZER_PIN);
        } else {
          lastNoteTime = now;
          if (marioNotes[marioNoteIndex] > 0) {
            tone(BUZZER_PIN, marioNotes[marioNoteIndex]);
          } else {
            noTone(BUZZER_PIN);
          }
        }
      }
    }
    
    // Handle vibrator pattern
    if (vibeActive) {
      unsigned long now = millis();
      unsigned long elapsed = now - vibeStart;
      unsigned long totalTime = 0;
      
      for (int i = 0; i < vibeStep; i++) {
        totalTime += vibePattern[i];
      }
      
      if (elapsed >= totalTime) {
        if (vibeStep < vibeLength) {
          if (vibePattern[vibeStep] > 0) {
            digitalWrite(VIBRATOR_PIN, HIGH);
          } else {
            digitalWrite(VIBRATOR_PIN, LOW);
            vibeActive = false;
          }
          vibeStep++;
        } else {
          digitalWrite(VIBRATOR_PIN, LOW);
          vibeActive = false;
        }
      }
    } else {
      digitalWrite(VIBRATOR_PIN, LOW);
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz update rate
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Full Dashboard Test");
  
  // Initialize I2C for battery fuel gauge
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz for IMU per schematic
  scanI2CDevices();
  delay(100); // Give I2C devices time to power up
  
  // Initialize battery gauge with fix
  batteryGaugeFound = checkBatteryGauge();
  if (batteryGaugeFound) {
    Serial.println("Battery gauge found!");
    // Initialize the MAX17048 properly
    if (initializeMAX17048()) {
      Serial.println("Battery gauge initialized successfully!");
    } else {
      Serial.println("Battery gauge initialization failed!");
      batteryGaugeFound = false;
    }
  } else {
    Serial.println("Battery gauge not found!");
  }
  
  // Play Mario coin sound at boot
  pinMode(BUZZER_PIN, OUTPUT);
  playMarioCoin();
  
  // Initialize IMU
  bool imuFound = initIMU();
  if (!imuFound) {
    Serial.println("IMU not found - orientation indicator will not work");
  }
  
  // Initialize display
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  
  // Initialize LEDs
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(128);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  
  // Initialize all buttons with pullups
  pinMode(BTN_L_SHOULDER, INPUT_PULLUP);
  pinMode(BTN_R_SHOULDER, INPUT_PULLUP);
  pinMode(BTN_UNDER_L, INPUT_PULLUP);
  pinMode(BTN_UNDER_C, INPUT_PULLUP);
  pinMode(BTN_UNDER_R, INPUT_PULLUP);
  pinMode(VIBRATOR_PIN, OUTPUT);
  
  // Initialize encoders with pullups
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  
  // Create mutex for encoder protection
  encoderMutex = xSemaphoreCreateMutex();
  
  // Attach interrupts to both A and B pins for better detection
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), handleLeftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), handleRightEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BTN_UNDER_L), leftButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_UNDER_R), rightButtonISR, FALLING);
  
  // Mark initialization complete
  dashboardReady = true;
  
  // Start dashboard task on core 1
  xTaskCreatePinnedToCore(
    dashboardTask,
    "Dashboard",
    4096,
    NULL,
    1,
    &dashboardTaskHandle,
    1  // Core 1
  );
  
  Serial.println("Setup complete");
}

void loop() {
  // Main loop only handles non-display tasks
  // All display updates are handled by the dashboard task
  
  // Read buttons (if needed)
  // Handle other I/O
  
  delay(100);
} 