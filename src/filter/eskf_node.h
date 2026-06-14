#pragma once

#include <stdint.h>

#include <rio/rio_eskf.h>
#include "drivers/xwr6843aop.h"

namespace eskf_node {

struct Stats {
  uint32_t imu, radar, baro;
  uint32_t baro_acc, baro_rej, baro_skp;
  float    baro_temp_c, baro_press_pa;
  bool     baro_valid;
};

void init();
bool isAttitudeInitialized();

void onImu(const rio::Vec3& acc, const rio::Vec3& gyr, float t);
void onRadarFrame(const RadarFrame& frame);
void onBaroSample(float temp_c, float press_pa, float t);
void onBaroReadFailed();

// Snapshot + reset counters atomically (called by debug's 1 Hz printer).
// Cached baro temp/press carry forward; only the counters reset.
Stats getAndResetStats();

}  // namespace eskf_node
