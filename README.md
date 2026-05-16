# ESP32-C3 Speedometer Reader

ESP-IDF firmware for an ESP32-C3 that reads a vehicle speed sensor pulse signal, calculates speed, smooths the result, and broadcasts it over ESP-NOW.

The firmware is intended for a small sender node: one GPIO watches the speed pulse input, the ESP32-C3 computes the current speed in 0.1 MPH or KPH units, and another ESP8266 or ESP32 receives the compact ESP-NOW packet.

## Features

- Interrupt-driven pulse timing on GPIO 5.
- ESP-NOW broadcast or targeted peer delivery.
- Fixed-point speed calculation and smoothing.
- Debounce/dead-zone filtering for noisy pulse edges.
- Physics-based rate limiting for impossible acceleration or deceleration spikes.
- Snap-to-zero behavior when pulses stop.
- Radio watchdog that reboots if ESP-NOW send confirmations stop for 10 seconds.
- Serial test mode that generates repeatable speed ramps without a connected sensor.
- ESP-IDF 5.x and 6.x compatible ESP-NOW send callback handling.

## Hardware

Required:

- ESP32-C3 development board.
- Vehicle speed sensor or compatible pulse source.
- Signal conditioning between the vehicle signal and ESP32-C3 GPIO.
- ESP-NOW receiver built with an ESP8266 or ESP32.

Default wiring:

| Signal | ESP32-C3 pin | Notes |
| --- | --- | --- |
| Speed pulse input | GPIO 5 | Configured as input with internal pull-up and falling-edge interrupt |
| Power | Board dependent | Use a regulated supply suitable for the ESP32-C3 board |
| Ground | GND | Common ground with the conditioned pulse signal |

Do not connect a vehicle VSS, 12 V signal, inductive sensor, or open-collector line directly to an ESP32-C3 pin. The ESP32-C3 GPIO input must be protected and limited to 3.3 V logic levels.

## Software

This is a plain ESP-IDF CMake project.

Tested build environment:

- Target: `esp32c3`
- ESP-IDF: 6.0.1
- Flash size default: 4 MB

The source includes compatibility handling for the ESP-NOW send callback API change between ESP-IDF 5.x and 6.x.

## Build And Flash

From the repository root:

```sh
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

If `idf.py` is not available in your shell, load your ESP-IDF environment first:

```sh
. /path/to/esp-idf/export.sh
```

The default sdkconfig values live in `sdkconfig.defaults`.

## Configuration

Runtime configuration is currently done with constants in [main/main.c](main/main.c).

### Radio

```c
#define WIFI_CHANNEL 1
static const uint8_t receiver_mac[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
```

The default MAC address is the ESP-NOW broadcast address. Replace it with a receiver MAC address to target one receiver. The sender and receiver must use the same Wi-Fi channel.

### Speed Input

```c
#define SPEED_PIN GPIO_NUM_5
#define SPEED_DEADZONE_US 2000UL
#define SNAP_TO_ZERO_US 500000UL
```

`SPEED_DEADZONE_US` rejects edges that arrive too soon after the previous accepted pulse. `SNAP_TO_ZERO_US` clears the pulse state after a long gap so the output can settle back to zero.

### Sampling And Output

```c
#define SAMPLE_INTERVAL_MS 100
#define OUTPUT_KPH 0
```

The firmware samples and transmits every 100 ms. Set `OUTPUT_KPH` to `1` to transmit KPH instead of MPH.

### Calibration

```c
static const uint32_t K_SPEED_X10 = 8779631UL;
```

`K_SPEED_X10` converts measured pulse period into speed in tenths of a unit. The default value is derived from `10,000,000 / 1.139`, where `1.139` is the current VSS calibration factor used by this project.

If the reading is consistently high or low, recalibrate this value for your pulse source and drivetrain.

### Smoothing

```c
#define FILTER_WEIGHT_NUM 4
#define FILTER_WEIGHT_DEN 10
```

The final output uses fixed-point exponential smoothing. The default ratio, `4 / 10`, applies a 0.40 weight to the newest speed sample.

## Test Mode

Open the serial monitor and send `t` or `T` to toggle test mode.

In test mode the firmware ignores the physical pulse input, clears accumulated pulse state, and emits repeatable ramp profiles:

- 30 seconds up/down to 120 MPH.
- 20 seconds up/down to 120 MPH.
- 15 seconds up/down to 60 MPH.

This is useful for validating an ESP-NOW receiver, display, or dashboard integration without spinning a sensor.

## ESP-NOW Payload

The sender transmits a 3-byte packed payload:

```c
typedef struct __attribute__((packed))
{
  uint8_t unit;   // 'M' for MPH, 'K' for KPH
  uint16_t speed; // speed in 0.1 unit increments
} speed_packet_t;
```

Examples:

- `unit = 'M'`, `speed = 657` means 65.7 MPH.
- `unit = 'K'`, `speed = 1062` means 106.2 KPH.

Receiver code should treat the packet as little-endian when reading `uint16_t speed` across platforms.

## Behavior Notes

- The pulse ISR and time wrapper are placed in IRAM-safe paths for reliable edge handling during flash/cache-sensitive operations.
- The periodic sample timer notifies the main task directly, avoiding a spinlock-protected tick counter.
- If the main task falls behind, queued sample notifications are capped at four per loop to avoid long catch-up bursts.
- ESP-NOW send failures are logged. If no successful send callback is observed for 10 seconds, the firmware restarts.

## Troubleshooting

No ESP-NOW packets received:

- Confirm sender and receiver are on the same Wi-Fi channel.
- Start with the broadcast MAC address, then switch to a fixed receiver MAC once basic reception works.
- Check that the receiver expects the 3-byte payload format above.

Speed reads zero:

- Verify the conditioned pulse reaches GPIO 5 as a 3.3 V logic signal.
- Confirm the pulse edge matches the falling-edge interrupt setup.
- Use test mode to separate sensor input issues from radio/receiver issues.

Speed is noisy or jumps:

- Increase hardware filtering or improve signal conditioning first.
- Tune `SPEED_DEADZONE_US` for contact bounce or electrical noise.
- Revisit `K_SPEED_X10` if readings are consistently scaled wrong.

Build cannot find `idf.py`:

- Source the ESP-IDF export script for your installation.
- Confirm `IDF_PATH` points to the intended ESP-IDF checkout.

## Project Layout

```text
.
├── CMakeLists.txt
├── LICENSE
├── README.md
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── sdkconfig.defaults
```

## License

This project is licensed under the GNU General Public License version 3.0 only. See [LICENSE](LICENSE) for the full license text.

GPLv3 allows you to use, study, modify, and redistribute the software under the terms of the license. If you distribute modified versions or binaries, you must comply with the GPLv3 source code and license notice requirements.

This software is provided without warranty; see the GPLv3 warranty disclaimer in [LICENSE](LICENSE).
