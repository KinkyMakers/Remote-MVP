#ifndef LSM6DS3_SIMPLE_H
#define LSM6DS3_SIMPLE_H

#include <Wire.h>

class LSM6DS3_Simple {
public:
    LSM6DS3_Simple() : _addr(0x6A), _wire(nullptr) {}
    
    // Call this in setup()
    bool begin(TwoWire &wire, uint8_t addr = 0x6A) {
        _wire = &wire;
        _addr = addr;
        // Check WHO_AM_I
        uint8_t who = readReg(0x0F);
        if (who != 0x69 && who != 0x6A) return false;
        // Explicit config for normal operation
        writeReg(0x10, 0x60); // CTRL1_XL: ODR_XL=208Hz, FS_XL=2g, BW_XL=100Hz
        writeReg(0x11, 0x60); // CTRL2_G: ODR_G=208Hz, FS_G=245dps
        writeReg(0x12, 0x44); // CTRL3_C: BDU=1, IF_INC=1
        writeReg(0x15, 0x00); // CTRL6_C: default
        delay(100); // Wait for sensor to be ready
        return true;
    }
    // Read acceleration in g
    void readAccel(float &x, float &y, float &z) {
        int16_t raw[3];
        readVec3(0x28, raw);
        // ±2g, 16-bit, 0.061 mg/LSB
        x = raw[0] * 0.000061f;
        y = raw[1] * 0.000061f;
        z = raw[2] * 0.000061f;
    }
    // Read gyro in dps
    void readGyro(float &x, float &y, float &z) {
        int16_t raw[3];
        readVec3(0x22, raw);
        // ±125 dps, 16-bit, 4.375 mdps/LSB
        x = raw[0] * 0.004375f;
        y = raw[1] * 0.004375f;
        z = raw[2] * 0.004375f;
    }
    // Read WHO_AM_I
    uint8_t whoami() { return readReg(0x0F); }
    // Read any register
    uint8_t readReg(uint8_t reg) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_addr, (uint8_t)1);
        if (_wire->available()) return _wire->read();
        return 0xFF;
    }
    // Write any register
    void writeReg(uint8_t reg, uint8_t val) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }
    // Print raw accel/gyro output registers
    void debugRaw() {
        uint8_t buf[12];
        readRawBlock(0x22, buf, 12);
        Serial.print("Raw Gyro: ");
        for (int i = 0; i < 6; ++i) Serial.printf("%02X ", buf[i]);
        Serial.print("| Raw Accel: ");
        for (int i = 6; i < 12; ++i) Serial.printf("%02X ", buf[i]);
        Serial.println();
    }
private:
    uint8_t _addr;
    TwoWire *_wire;
    // Read 3x int16_t from consecutive registers (little endian)
    void readVec3(uint8_t base, int16_t *out) {
        for (int i = 0; i < 3; ++i) {
            uint8_t l = readReg(base + i * 2);
            uint8_t h = readReg(base + i * 2 + 1);
            out[i] = (int16_t)(l | (h << 8));
        }
    }
    void readRawBlock(uint8_t base, uint8_t *buf, uint8_t len) {
        for (uint8_t i = 0; i < len; ++i) {
            buf[i] = readReg(base + i);
        }
    }
};

#endif // LSM6DS3_SIMPLE_H 