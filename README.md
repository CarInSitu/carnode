# Car Node

## Wiring

| Wemos D1 mini GPIO | ESP8266 GPIO | Connected to   | Notes                                                                 |
|--------------------|--------------|----------------|-----------------------------------------------------------------------|
| D1                 | 5            | Steering Servo | PPM (Pulse-position modulation)                                       |
| D2                 | 4            | Throttle ESC   | PPM (Pulse-position modulation)                                       |
| D5                 | 14           | IR receiver    | TSOP3xx series                                                        |
| D7, D8             | 13, 15       | SmartAudio     | Both ESP pins are connected to the SmartAudio pin (ie. short-circuit) |

## Installation

Follow instructions at https://github.com/esp8266/Arduino#installing-with-boards-manager
