# Battery Fuel Gauge Troubleshooting Guide

## Problem Description
The battery fuel gauge (MAX17048) is stuck at 100% and 2.56V, indicating incorrect readings.

## Root Cause Analysis

### 1. Hardware Issues
- **I2C Connection**: The MAX17048 is connected to pins 8 (SDA) and 9 (SCL)
- **Power Supply**: The fuel gauge requires proper 3.3V supply
- **Pull-up Resistors**: I2C bus may need external pull-up resistors

### 2. Software Issues
- **Incorrect Register Reading**: The code may be reading wrong registers
- **Wrong I2C Address**: The device might be at a different address
- **Incorrect Data Conversion**: Voltage/SOC conversion formulas may be wrong
- **Stuck Readings**: The fuel gauge may need a reset

## Troubleshooting Steps

### Step 1: Run the Diagnostic Script
1. Upload `battery_debug.cpp` to your device
2. Open Serial Monitor at 115200 baud
3. Check the output for:
   - I2C device detection
   - Register values
   - Communication errors

### Step 2: Check Hardware Connections
1. **Verify I2C Pins**: Ensure pins 8 (SDA) and 9 (SCL) are properly connected
2. **Check Power**: Verify 3.3V supply to the MAX17048
3. **Pull-up Resistors**: Add 4.7kΩ pull-up resistors to SDA and SCL if not present
4. **Ground Connection**: Ensure proper ground connection

### Step 3: Test Different I2C Addresses
The MAX17048 might be at different addresses:
- Primary: 0x36
- Alternative 1: 0x6C
- Alternative 2: 0x6D

### Step 4: Reset the Fuel Gauge
The MAX17048 may have stuck readings that need a reset:
1. Send reset command to register 0xFE
2. Wait for reset to complete
3. Re-read values

### Step 5: Verify Register Reading
The correct registers for MAX17048 are:
- **VCELL (Voltage)**: Register 0x02 (12-bit, 1.25mV/LSB)
- **SOC (Percentage)**: Register 0x04 (16-bit, 1/256% resolution)
- **Version**: Register 0x08
- **Config**: Register 0x0C

## Solutions

### Solution 1: Use the Fixed Battery Monitor Class
Replace the current battery reading code with the `BatteryMonitor` class from `battery_fix.h`:

```cpp
#include "battery_fix.h"

BatteryMonitor battery;

void setup() {
    if (battery.begin(8, 9)) {
        Serial.println("Battery monitor initialized!");
        battery.runDiagnostic();
    }
}

void loop() {
    float voltage = battery.readVoltage();
    int percent = battery.readPercentage();
    // Use the values...
}
```

### Solution 2: Manual I2C Testing
If the class doesn't work, use the diagnostic script to:
1. Find the correct I2C address
2. Verify register values
3. Test different conversion formulas

### Solution 3: Hardware Fixes
1. **Add Pull-up Resistors**: Connect 4.7kΩ resistors from SDA and SCL to 3.3V
2. **Check Power Supply**: Ensure stable 3.3V supply
3. **Verify Connections**: Check for loose or broken connections

## Expected Results

### Normal Operation
- **Voltage**: Should read between 2.5V and 4.2V for Li-ion battery
- **SOC**: Should read between 0% and 100%
- **Version Register**: Should read 0x10 or similar
- **Config Register**: Should be non-zero

### Debug Output
The diagnostic scripts will show:
- Raw register values
- Calculated voltage and percentage
- I2C communication status
- Device detection results

## Common Issues and Fixes

### Issue: No I2C Devices Found
**Fix**: Check hardware connections and power supply

### Issue: Device Found but Wrong Values
**Fix**: Try different I2C addresses or reset the fuel gauge

### Issue: Stuck at 100% and 2.56V
**Fix**: This indicates incorrect register reading or conversion - use the fixed code

### Issue: Communication Errors
**Fix**: Add pull-up resistors or reduce I2C clock speed

## Testing Commands

### Run Diagnostic
```bash
# Upload and run the diagnostic script
pio run -t upload -e battery_debug
```

### Run Fixed Code
```bash
# Upload and run the fixed battery monitor
pio run -t upload -e battery_test
```

## Next Steps

1. **Run the diagnostic script** to identify the specific issue
2. **Check hardware connections** if no devices are found
3. **Try the fixed battery monitor class** if devices are found but readings are wrong
4. **Add pull-up resistors** if communication is unstable
5. **Contact hardware support** if issues persist

## Files Created

- `battery_debug.cpp`: Comprehensive diagnostic script
- `battery_fix.h`: Fixed battery monitor class
- `battery_test.cpp`: Simple test script
- `BATTERY_TROUBLESHOOTING.md`: This guide

## Additional Resources

- MAX17048 Datasheet: Check for correct register addresses and conversion formulas
- I2C Bus Troubleshooting: Verify bus integrity and timing
- ESP32 I2C Configuration: Ensure proper pin configuration 