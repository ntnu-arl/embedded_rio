#include "io/profiling.h"

#if PROFILING_ENABLED

#include <Arduino.h>
#include <string.h>

namespace profiling {
namespace {

struct Acc {
  uint64_t sum_cycles;
  uint32_t count;
  uint32_t max_cycles;
};

Acc s_acc[NUM_SECTIONS] = {};

const char* const s_names[NUM_SECTIONS] = {
    "imu_predict",
    "radar_point",
    "radar_frame",
    "baro_update",
    "mavlink_publish",
    "super_loop",
};

}  // namespace

void init() {
  // Enable the DWT cycle counter that cycles() reads.
  ARM_DEMCR    |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  // Bring up USB serial for the report. Do NOT wait for a host (must never
  // block in flight); prints are simply dropped until USB enumerates.
  Serial.begin(115200);
}

void add(Section s, uint32_t t0) {
  const uint32_t dt = ARM_DWT_CYCCNT - t0;  // uint32 subtraction is wrap-safe
  Acc& a = s_acc[s];
  a.sum_cycles += dt;
  a.count++;
  if (dt > a.max_cycles) a.max_cycles = dt;
}

void report(uint32_t now_ms) {
  static uint32_t s_last_ms = 0;
  if (now_ms - s_last_ms < 1000) return;
  s_last_ms = now_ms;

  // Runtime clock rate (Teensy 4.x F_CPU_ACTUAL = 600 MHz unless overclocked).
  const float cyc_per_us = F_CPU_ACTUAL / 1000000.0f;

  Serial.println("PROFILE us  (avg | n | max)");
  for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
    const Acc& a = s_acc[i];
    const float avg_us = (a.count > 0)
        ? static_cast<float>(static_cast<double>(a.sum_cycles) / a.count / cyc_per_us)
        : 0.0f;
    const float max_us = a.max_cycles / cyc_per_us;

    Serial.print("  ");
    Serial.print(s_names[i]);
    for (int p = static_cast<int>(strlen(s_names[i])); p < 16; p++) Serial.print(' ');
    Serial.print(avg_us, 2);
    Serial.print(" | ");
    Serial.print(a.count);
    Serial.print(" | ");
    Serial.println(max_us, 2);
  }
}

}  // namespace profiling

#endif  // PROFILING_ENABLED
