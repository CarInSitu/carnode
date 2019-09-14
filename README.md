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
