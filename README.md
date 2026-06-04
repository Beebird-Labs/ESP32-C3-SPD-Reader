# _CAUTION: THIS CODE IS WIP AND IS NOT GUARANTEED TO WORK IN ITS CURRENT STATE_

[![Build](https://github.com/Beebird-Labs/ESP32-C3-SPD-Reader/actions/workflows/build.yml/badge.svg)](https://github.com/Beebird-Labs/ESP32-C3-SPD-Reader/actions/workflows/build.yml)

# ESP32-C3-SPD-Reader

## Overview

ESP-IDF firmware for an ESP32-C3 that reads a vehicle speed sensor pulse signal, calculates speed, smooths the result, and sends it through the current speed-output backend.

The current firmware is intended for a small sender node: one GPIO watches the speed pulse input, the ESP32-C3 computes the current speed in 0.1 MPH or KPH units, and another ESP8266 or ESP32 receives the compact ESP-NOW packet. The output backend is isolated so the delivery path can move to CAN bus without changing speed capture or smoothing code.

## Features

- Interrupt-driven pulse timing on GPIO 3.
- Replaceable speed-output backend, currently ESP-NOW broadcast or targeted peer delivery.
- Fixed-point speed calculation and smoothing.
- ESP32-C3 GPIO glitch filtering plus firmware dead-zone filtering for noisy pulse edges.
- Physics-based rate limiting for impossible acceleration or deceleration spikes.
- Snap-to-zero behavior when pulses stop.
- OLED status display with a waiting animation and one-second speed updates.
- Speed-output watchdog that reboots if send confirmations stop for 10 seconds.
- Serial test mode that generates repeatable speed ramps without a connected sensor.
- ESP-IDF 5.x and 6.x compatible ESP-NOW send callback handling.

## Hardware

Required hardware:

- ESP32-C3 development board.
- Vehicle speed sensor or compatible pulse source.
- Signal conditioning between the vehicle signal and ESP32-C3 GPIO.
- SSD1306-compatible 72 x 40 I2C OLED display at address `0x3C`.
- ESP-NOW receiver built with an ESP8266 or ESP32.

Default wiring:

| Signal | ESP32-C3 pin | Notes |
| --- | --- | --- |
| Speed pulse input | GPIO 3 | Input with internal pull-up and falling-edge interrupt |
| OLED SDA | GPIO 5 | I2C data |
| OLED SCL | GPIO 6 | I2C clock |
| Power | Board dependent | Use a regulated supply suitable for the ESP32-C3 board |
| Ground | GND | Required common ground with the conditioned pulse signal |

Do not connect a vehicle VSS, 12 V signal, inductive sensor, or open-collector line directly to an ESP32-C3 pin. The ESP32-C3 GPIO input must be protected and limited to 3.3 V logic levels, and the pulse source must share a ground reference with the ESP32-C3.

## Software

This is a plain ESP-IDF CMake project.

Tested build environment:

- Target: `esp32c3`
- ESP-IDF: `6.0.1`
- Flash size default: 4 MB

The source includes compatibility handling for the ESP-NOW send callback API change between ESP-IDF 5.x and 6.x.

The application is split into focused modules:

- `main.c` owns speed pulse capture, speed calculation, smoothing, test mode, and top-level orchestration.
- `speed_output.c` owns the current ESP-NOW speed-output backend, payload packing, peer setup, send callback, and output watchdog timestamp. This is the narrow module boundary intended to make a future CAN bus backend replacement cleaner.
- `ble_prov.c` owns the BLE provisioning GATT service used to enter Wi-Fi credentials and trigger OTA.
- `ota_manager.c` owns OTA credential storage, update download, rollback validation, and OTA task lifetime.
- `wifi_sta.c` owns temporary Wi-Fi station connection for OTA.
- `oled.c` owns the SSD1306 display driver and display buffer.

## Build

From the repository root:

```sh
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

The release-facing app binary is generated as:

```text
build/ESP32-C3-SPD-Reader.bin
```

The OTA URL in `main/project_config.h` expects the release asset to use that
same filename.

If `idf.py` is not available in your shell, load your ESP-IDF environment first:

```sh
. /path/to/esp-idf/export.sh
```

The shared sdkconfig base lives in `sdkconfig.defaults`.

Optional build overlays:

- `sdkconfig.dev.defaults` for bring-up and debugging.
- `sdkconfig.release.defaults` for size-oriented release builds.

Use them with `SDKCONFIG_DEFAULTS`, for example:

```sh
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.dev.defaults" idf.py build
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.release.defaults" idf.py build
```

## Configuration

Runtime configuration is centralized in [main/project_config.h](main/project_config.h).

### Speed Output

```c
#define APP_WIFI_CHANNEL 1
#define APP_RECEIVER_MAC_BYTES 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
```

The current backend is ESP-NOW. The default MAC address is the ESP-NOW broadcast address. Replace it with a receiver MAC address to target one receiver. The sender and receiver must use the same Wi-Fi channel.

### Speed Input

```c
#define APP_SPEED_PIN GPIO_NUM_3
#define APP_SPEED_DEADZONE_US 2000UL
#define APP_SNAP_TO_ZERO_US 500000UL
#define APP_MAX_INPUT_SPEED_X10 1220
```

The ESP32-C3 pin glitch filter is enabled before the speed ISR is attached. `APP_SPEED_DEADZONE_US` rejects edges that arrive too soon after the previous accepted pulse. `APP_MAX_INPUT_SPEED_X10` rejects pulse intervals that imply an impossible speed, which helps suppress extra edges from noise or ringing. `APP_SNAP_TO_ZERO_US` clears the pulse state after a long gap so the output can settle back to zero.

### Sampling And Output

```c
#define APP_SAMPLE_INTERVAL_MS 100
#define APP_OUTPUT_KPH 0
#define APP_SPEED_OUTPUT_WATCHDOG_MS 10000UL
```

The firmware samples and transmits every 100 ms. Set `APP_OUTPUT_KPH` to `1` to transmit KPH instead of MPH. `APP_SPEED_OUTPUT_WATCHDOG_MS` controls how long the current output backend can go without a successful send confirmation before the firmware restarts.

### Calibration

```c
#define APP_K_SPEED_X10 8876731UL
```

`APP_K_SPEED_X10` converts measured pulse period into speed in tenths of a unit. The default value is calibrated from `70 Hz = 100 KPH`, which is `62.1371 MPH`.

### Smoothing

```c
#define APP_FILTER_WEIGHT_NUM 4
#define APP_FILTER_WEIGHT_DEN 10
```

The final output uses fixed-point exponential smoothing. The default ratio, `4 / 10`, applies a 0.40 weight to the newest speed sample.

## Protocol

### BLE Provisioning Service

The BLE device advertises as `SPD-Reader` and exposes a provisioning service for OTA setup:

| UUID | Access | Purpose |
| --- | --- | --- |
| `E8F00001-1B25-4F47-82AB-DE1E2AB9A87C` | service | provisioning service |
| `E8F00002-1B25-4F47-82AB-DE1E2AB9A87C` | write | Wi-Fi SSID |
| `E8F00003-1B25-4F47-82AB-DE1E2AB9A87C` | write | Wi-Fi password |
| `E8F00004-1B25-4F47-82AB-DE1E2AB9A87C` | write | trigger OTA update |
| `E8F00005-1B25-4F47-82AB-DE1E2AB9A87C` | read, notify | OTA status text |

BLE provisioning is disabled once the vehicle is moving so OTA cannot be started while speed output is active.

### Test Mode

Open the serial monitor and send `t` or `T` to toggle test mode.

In test mode the firmware ignores the physical pulse input, clears accumulated pulse state, and emits repeatable ramp profiles:

- 30 seconds up/down to 120 MPH
- 20 seconds up/down to 120 MPH
- 15 seconds up/down to 60 MPH

This is useful for validating an ESP-NOW receiver, display, or dashboard integration without spinning a sensor.

### ESP-NOW Payload

The current `speed_output.c` backend transmits a 3-byte packed ESP-NOW payload:

```c
typedef struct __attribute__((packed))
{
  uint8_t unit;
  uint16_t speed;
} speed_packet_t;
```

- `unit = 'M'`, `speed = 657` means 65.7 MPH
- `unit = 'K'`, `speed = 1062` means 106.2 KPH

Receiver code should treat the packet as little-endian when reading `uint16_t speed` across platforms.

The speed calculation code calls the `speed_output` API instead of calling ESP-NOW directly. The current backend is ESP-NOW, but the API boundary exists so a future CAN bus backend can replace the delivery path without changing pulse capture, smoothing, OLED display, BLE provisioning, or OTA code.

## Runtime Behavior

- The pulse ISR and time wrapper are placed in IRAM-safe paths for reliable edge handling during flash/cache-sensitive operations.
- The OLED shows `Waiting` with animated dots at boot, then displays speed once pulses are observed.
- The periodic sample timer notifies the main task directly, avoiding a spinlock-protected tick counter.
- If the main task falls behind, queued sample notifications are capped at four per loop to avoid long catch-up bursts.
- Speed output send failures are logged. In the current ESP-NOW backend, if no successful send callback is observed for 10 seconds, the firmware restarts.
- Startup failures are logged and followed by restart instead of aborting immediately.

## Troubleshooting

No ESP-NOW packets received:

- Confirm sender and receiver are on the same Wi-Fi channel.
- Start with the broadcast MAC address, then switch to a fixed receiver MAC once basic reception works.
- Check that the receiver expects the 3-byte payload format above.

Speed reads zero:

- Verify the conditioned pulse reaches GPIO 3 as a 3.3 V logic signal.
- Confirm the pulse edge matches the falling-edge interrupt setup.
- Use test mode to separate sensor input issues from radio or receiver issues.

Speed is noisy or jumps:

- Confirm the pulse source and ESP32-C3 share a common ground.
- Increase hardware filtering or improve signal conditioning first.
- Tune `APP_SPEED_DEADZONE_US` for contact bounce or electrical noise.
- Revisit `APP_K_SPEED_X10` if readings are consistently scaled wrong.

Build cannot find `idf.py`:

- Source the ESP-IDF export script for your installation.
- Confirm `IDF_PATH` points to the intended ESP-IDF checkout.

## Project Layout

```text
.
|-- CMakeLists.txt
|-- LICENSE
|-- README.md
|-- partitions.csv
|-- main
|   |-- ble_prov.c
|   |-- ble_prov.h
|   |-- CMakeLists.txt
|   |-- main.c
|   |-- oled.c
|   |-- oled.h
|   |-- ota_manager.c
|   |-- ota_manager.h
|   |-- speed_output.c
|   |-- speed_output.h
|   |-- wifi_sta.c
|   |-- wifi_sta.h
|   `-- project_config.h
|-- sdkconfig.defaults
|-- sdkconfig.dev.defaults
`-- sdkconfig.release.defaults
```

## License

This project is licensed under the GNU General Public License version 3.0 only. See [LICENSE](LICENSE) for the full license text.
