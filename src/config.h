#pragma once

// #ifndef guard lets the replay ablation build override this with a -D compile
// flag (see tools/replay/CMakeLists.txt RIO_BARO_AIDING) without editing here.

// Set to 0 to send data to mavlink
// Set to 1 to send data to usb
#define USB_PRINT_ENABLED 0

// Runtime profiling of the ESKF pipeline (see io/profiling.h).
#define PROFILING_ENABLED 0

// Aiding switches. Set to 0 to read the sensor but skip its ESKF update —
#define RADAR_AIDING_ENABLED 1

#ifndef BARO_AIDING_ENABLED
#define BARO_AIDING_ENABLED  1
#endif

// Baro measurement mode:
//   1 = differential (re-anchor on each accept. Not used in the thesis)
//   0 = absolute     (anchor fixed at boot; z=0 = position at first baro lock)
#define BARO_AIDING_DIFFERENTIAL 0

// Radar second-order measurement underweighting 
#ifndef RADAR_UNDERWEIGHTING_ENABLED
#define RADAR_UNDERWEIGHTING_ENABLED 1
#endif

// SD card logging 
#define SD_LOG_ENABLED 1

// Hardware: 1 = Teensy 4.1 built-in SDIO (BUILTIN_SDCARD)
//           0 = Teensy 4.0 external SPI (specify CS pin below)
#ifndef SD_BUILTIN
#define SD_BUILTIN 1
#endif
#define SD_CS_PIN  10   // SPI chip-select pin for external SD (Teensy 4.0)

// Per-topic max log rate in Hz. 0 means log every sample.
#define SD_LOG_IMU_HZ   0
#define SD_LOG_RADAR_HZ 0
#define SD_LOG_BARO_HZ  0

// How often to flush buffered data to the SD card (seconds).
#define SD_LOG_FLUSH_INTERVAL_S 1
