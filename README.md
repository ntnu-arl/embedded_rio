# Embedded-RIO

Tightly-coupled radar-inertial odometry firmware for a Teensy 4.x. An
error-state Kalman filter (ESKF) fuses mmWave radar Doppler with IMU (and
optionally barometer) to estimate pose, and streams it to an unmodified PX4
flight controller over MAVLink (the external-vision interface) for flight in
GNSS-denied environments.

Hardware: TI IWR6843AOP radar, Bosch BMI088 IMU, Bosch BMP388 baro.

Datasheets:
[BMI088 (IMU)](https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi088) ·
[BMP388 (baro)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf) ·
[IWR6843AOP (radar)](https://www.ti.com/tool/IWR6843AOPEVM)

## Build & flash (firmware)

Built with [PlatformIO](https://platformio.org/). `teensy40` is the active
target (the `teensy41` environment is currently commented out in
[platformio.ini](platformio.ini)).

```sh
pio run -e teensy40                  # build firmware
pio run -e teensy40 --target upload  # flash over USB
```

## config.h

[src/config.h](src/config.h) holds the compile-time flags, and is **shared by
both the firmware and the offline replay tool**. Edit it and rebuild to change
behaviour. The main flags:

| Flag | Meaning |
| --- | --- |
| `RADAR_AIDING_ENABLED` | Master radar on/off (read the sensor but skip its ESKF update when 0) |
| `BARO_AIDING_ENABLED` | Master barometer on/off |
| `BARO_AIDING_DIFFERENTIAL` | `0` = absolute anchor (default), `1` = differential re-anchoring |
| `RADAR_UNDERWEIGHTING_ENABLED` | Radar second-order measurement underweighting |
| `USB_PRINT_ENABLED` | `1` = print state over USB serial, `0` = send over MAVLink |
| `SD_LOG_ENABLED` / `SD_BUILTIN` | SD logging; `SD_BUILTIN` = `1` Teensy 4.1 SDIO, `0` Teensy 4.0 SPI |
| `PROFILING_ENABLED` | Runtime profiling of the ESKF pipeline |

The `#ifndef`-guarded flags (`BARO_AIDING_ENABLED`, `RADAR_UNDERWEIGHTING_ENABLED`,
`SD_BUILTIN`) can be overridden at build time with `-D` flags (this is how the
replay ablation study sweeps configurations without editing this file).

## Live plotting

Set `#define USB_PRINT_ENABLED 1` in [src/config.h](src/config.h) and reflash.
The firmware then streams the full ESKF state over USB serial, which you can
visualize live:

```sh
pip install pyserial matplotlib numpy
python3 scripts/serial_plotter.py --port /dev/ttyACM0
```

## Offline replay

The replay tool runs the same ESKF code as the firmware against a recorded
CSV log on your desktop (for tuning, validation, and ablation studies). The
[Makefile](Makefile) wraps the common workflow:

```sh
make build   
make run       
make compare   
make ablation   # plots all permutations
```

Inputs default from the `RIO_LOG` / `RIO_ULG` environment variables.

Run `make help` for the full list of targets, inputs, and comparison flags.
