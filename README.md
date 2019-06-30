# Car Node

## Wiring

| Wemos D1 mini GPIO | ESP8266 GPIO | Connected to   | Notes                                                                 |
|--------------------|--------------|----------------|-----------------------------------------------------------------------|
| D1                 | 5            | Steering Servo | PPM (Pulse-position modulation)                                       |
| D2                 | 4            | Throttle ESC   | PPM (Pulse-position modulation)                                       |
| D5                 | 14           | IR receiver    | TSOP3xx series                                                        |
| D7, D8             | 13, 15       | SmartAudio     | Both ESP pins are connected to the SmartAudio pin (ie. short-circuit) |

## Notes

Our current VTX (ie. FX868T) seems to not support 'Set Frequency' function specified in SmartAudio datasheet : it correctly reply to request but no change appears while setting channel fully works.

## Installation

Follow instructions at https://github.com/esp8266/Arduino#installing-with-boards-manager
