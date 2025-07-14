# OSSM-Wireless
Wireless OSSM Remote



| GPIO # | Function | Note |
|--------|----------|------|
| 1 | Right Shoulder Button| |
| 2 | Buzzer | Use PWM, not digital on/off |
| 4 | Screen Cable Select | |
| 5 | Screen Clock | |
| 6 | Screen Data | |
| 7 | Screen A0 | |
| 8 | SDA | Should be default pin for ESP32-S3 |
| 9 | SCL | Should be default pin for ESP32-S3 |
| 13 | Gyro Int 2 | Interrupt from Gyro *1*|
| 14 | Gyro Int 1 | Interrupt from Gyro *1*|
| 15 | Screen Backlight | Can PWM for dimming |
| 16 | Screen Reset | |
| 17 | | |
| 18 | | |
| 19 | External Pin 2 | Alternative: i2c, USB (D+) - has pull up switch on PCB|
| 20 | External Pin 1 | Alternative: i2c, USB (D-)- has pull up switch on PCB|
| 21 | 3v3 External | Turn on to allow accessories to receive 3.3v |
| 35 | Left Encoder B | |
| 36 | Left Encoder A | |
| 37 | LED Pixels | |
| 38 | Under Screen Left Button | |
| 39 | Under Screen Centre Button | |
| 40 | Under Screen Right Button | |
| 41 | Right Encoder B | |
| 42 | Right Encoder A | |
| 47 | Vibrator | Only run for short duration and PWM for speed control |
| 48 | Left Shoulder Button | |


## Fuel Guage

The battery fuel guage is a MAX17048G+T10 - LCSC Part # C2682616

The connection to the ESP32 is through the i2c on the board, pins # IO8 (SDA) and # IO9 (SCL)

## Gyro / Accelerometer

The chip is a LSM6DS3TR-C - LCSC Part # C967633

Again, this is connected to the internal i2c bus. There is the addition of the interrupts connected to IO13 (int2) and IO14 (int1)

## screen 

is a 2 inch SPI ST7789, the pins are in the chart above









