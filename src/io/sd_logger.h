#pragma once

#include "config.h"

#if SD_LOG_ENABLED

#include <rio/rio_eskf.h>
#include "drivers/xwr6843aop.h"

// Call once in setup(). Opens LOG0000.CSV (or next available index).
// Returns false if SD init or file open fails.
bool sdLoggerInit();

// Log one IMU sample. Rate-limited to SD_LOG_IMU_HZ.
void sdLoggerLogImu(float t_s, const rio::Vec3& acc, const rio::Vec3& gyr);

// Log one radar frame (one row per point). Rate-limited to SD_LOG_RADAR_HZ.
void sdLoggerLogRadar(float t_s, const RadarFrame& frame);

// Log one barometer reading. Rate-limited to SD_LOG_BARO_HZ.
void sdLoggerLogBaro(float t_s, float temp_c, float press_pa);

// Flush buffered data to card. Call at ~1 Hz.
void sdLoggerFlush();

#endif // SD_LOG_ENABLED
