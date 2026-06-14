#include "io/sd_logger.h"

#if SD_LOG_ENABLED

#include <SD.h>
#include <stdio.h>

// ── rate-limit periods ────────────────────────────────────────────────────────

static constexpr uint32_t periodUs(uint32_t hz) {
    return (hz == 0) ? 0 : (1000000UL / hz);
}

static constexpr uint32_t IMU_PERIOD_US   = periodUs(SD_LOG_IMU_HZ);
static constexpr uint32_t RADAR_PERIOD_US = periodUs(SD_LOG_RADAR_HZ);
static constexpr uint32_t BARO_PERIOD_US  = periodUs(SD_LOG_BARO_HZ);

// ── state ─────────────────────────────────────────────────────────────────────

static bool     s_ready         = false;
static File     s_file;
static uint32_t s_imu_last_us   = 0;
static uint32_t s_radar_last_us = 0;
static uint32_t s_baro_last_us  = 0;

// ── init ──────────────────────────────────────────────────────────────────────

bool sdLoggerInit() {
#if SD_BUILTIN
    const bool sd_ok = SD.begin(BUILTIN_SDCARD);
#else
    const bool sd_ok = SD.begin(SD_CS_PIN);
#endif
    if (!sd_ok) {
#if USB_PRINT_ENABLED
        Serial.println("SD init failed");
#endif
        return false;
    }

    // Find the first unused LOG####.CSV filename.
    char name[16];
    uint16_t idx = 0;
    do {
        snprintf(name, sizeof(name), "LOG%04u.CSV", idx++);
    } while (SD.exists(name) && idx <= 9999);

    s_file = SD.open(name, FILE_WRITE);
    if (!s_file) {
#if USB_PRINT_ENABLED
        Serial.println("SD file open failed");
#endif
        return false;
    }

    // Single header covers all row types.
    // IMU:   v0-v5 = ax, ay, az, gx, gy, gz
    // RADAR: v0-v4 = pt, x,  y,  z,  vr  (v5 empty)
    // BARO:  v0-v1 = temp_c, press_pa     (v2-v5 empty)
    s_file.println("type,t_s,v0,v1,v2,v3,v4,v5");

    s_ready = true;
#if USB_PRINT_ENABLED
    Serial.print("SD logging to ");
    Serial.println(name);
#endif
    return true;
}

// ── log functions ─────────────────────────────────────────────────────────────

void sdLoggerLogImu(float t_s, const rio::Vec3& acc, const rio::Vec3& gyr) {
    if (!s_ready) return;
    const uint32_t now = micros();
    if (IMU_PERIOD_US > 0 && (now - s_imu_last_us) < IMU_PERIOD_US) return;
    s_imu_last_us = now;

    s_file.print("IMU,");
    s_file.print(t_s, 6);
    s_file.print(','); s_file.print(acc.x(), 6);
    s_file.print(','); s_file.print(acc.y(), 6);
    s_file.print(','); s_file.print(acc.z(), 6);
    s_file.print(','); s_file.print(gyr.x(), 6);
    s_file.print(','); s_file.print(gyr.y(), 6);
    s_file.print(','); s_file.println(gyr.z(), 6);
}

void sdLoggerLogRadar(float t_s, const RadarFrame& frame) {
    if (!s_ready || !frame.valid || frame.numRaw == 0) return;
    const uint32_t now = micros();
    if (RADAR_PERIOD_US > 0 && (now - s_radar_last_us) < RADAR_PERIOD_US) return;
    s_radar_last_us = now;

    for (uint32_t i = 0; i < frame.numRaw; i++) {
        s_file.print("RAD,");
        s_file.print(t_s, 6);
        s_file.print(','); s_file.print(i);
        s_file.print(','); s_file.print(frame.raw[i].x, 6);
        s_file.print(','); s_file.print(frame.raw[i].y, 6);
        s_file.print(','); s_file.print(frame.raw[i].z, 6);
        s_file.print(','); s_file.println(frame.raw[i].vr, 6);
    }
}

void sdLoggerLogBaro(float t_s, float temp_c, float press_pa) {
    if (!s_ready) return;
    const uint32_t now = micros();
    if (BARO_PERIOD_US > 0 && (now - s_baro_last_us) < BARO_PERIOD_US) return;
    s_baro_last_us = now;

    s_file.print("BAR,");
    s_file.print(t_s, 6);
    s_file.print(','); s_file.print(temp_c, 6);
    s_file.print(','); s_file.println(press_pa, 6);
}

// ── flush ─────────────────────────────────────────────────────────────────────

void sdLoggerFlush() {
    if (s_ready) s_file.flush();
}

#endif // SD_LOG_ENABLED
