# Car Node

This firmware is designed to be embedded in _Car In Situ_ RC car.

Although the main goal is to use _CarNode_ with a camera, this one is optionnal.

_CarNode_ allows to use a _Wemos D1 mini_ board, or similar, to turn an RC car into a WiFi-driveable one with this GPL licenced firmware.

Note: A fully featured dedicated board exists for this project to easily convert a _WLtoys K989_ RC car.

## Features

 * Discoverable over MDNS
 * Discover and automatically connect to _CarInSitu_ server
 * Send flow commands (ie. steering and throttle) through UDP
 * Send others commands through TCP
 * Drive PPM servos
 * Talk with VTX to setup embedded camera (e.g. Video channels)
 * Over-The-Air firmware uploads
 * Decode IR codes and send them to the server

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

[0] APA106 LEDs behind a level shifter (LVC1T45)
[1] Selected, but not tested, I²C devices are:
 * Battery monitor (STC3100)
 * IMU (LSM6DS33)

## Development environment

### Setup platformio

```shell
pip install --user virtualenv
```

Add in `~/.profile`:

```shell
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi

if [ -d "$HOME/.platformio/penv/bin" ] ; then
    PATH="$HOME/.platformio/penv/bin:$PATH"
fi
```

Reload `~/.profile`:

```shell
source ~/.profile
```

```shell
virtualenv $HOME/.platformio/penv
```

### Usage

 * Compile

     ```
     pio run
     ```

 * Compile and upload through serial

    ```
    pio run -t upload
    ```

 * Compile and upload through OTA

    ```
    avahi-browse -a -t # List available mDNS services
    avahi-browse -t _arduino._tcp | grep CarNode | awk -F' ' '{ print $4 }' # List car nodes mDNS hostnames
    pio run --target upload --upload-port TARGET_FQDN.local
    ```

    If you want a massive OTA update:

    ```
    for i in `avahi-browse -t _arduino._tcp | grep CarNode | awk -F' ' '{ print $4 }'`; do
      pio run --target upload --upload-port ${i}.local
    done
    ```

 * Monitor (ie. connect to serial)

    ```
    pio device monitor
    ```

 * List available devices

    ```
    pio device list
    ```
