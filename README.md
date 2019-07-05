# Car Node

## Wiring

| Wemos D1 mini GPIO | ESP8266 GPIO | Connected to             | Notes                                |
|--------------------|--------------|--------------------------|--------------------------------------|
| D0                 | 16           |                          | /!\ deep sleep                       |
| D1                 | 5            | Steering Servo           | PPM (Pulse-position modulation)      |
| D2                 | 4            | Throttle ESC             | PPM (Pulse-position modulation)      |
| D3                 | 0            | Front headlights         | /!\ Boot mode (1: run / 0: flash)    |
| D4                 | 2            | Back and top headlights⁰ | /!\ Must be high at boot (boot mode) |
| D5                 | 14           | IR receiver              | TSOP3xx series                       |
| D6                 | 12           | I²C devices¹             | I2C SDA                              |
| D7                 | 13           | I²C devices¹             | I2C CLK                              |
| D8                 | 15           | VTX                      | SmartAudio pin                       |

[0] 3 LEDs APA106 behind a level shifter (LVC1T45)
[1] I²C devices are:
 * Battery monitor (STC3100)
 * IMU (LSM6DS33)

## Notes

Our current VTX (ie. FX868T) seems to not support 'Set Frequency' function specified in SmartAudio datasheet : it correctly reply to request but no change appears while setting channel fully works.

## Installation

Follow instructions at https://github.com/esp8266/Arduino#installing-with-boards-manager
