#pragma once

#include <stdint.h>

#include "config.h"

#if PROFILING_ENABLED
#include <Arduino.h>  // ARM_DWT_CYCCNT and register macros
#endif

// Lightweight wall-clock profiler for the ESKF pipeline. Each timed section
// keeps a running (cumulative) average, sample count, and worst-case duration,
// printed to USB Serial once per second by report(). Timing uses the ARM cycle
// counter (ARM_DWT_CYCCNT, 600 MHz on Teensy 4.x) for sub-µs resolution.
//
// Usage:
//   { PROFILE_SCOPE(IMU_PREDICT); eskf.predict(...); eskf.insPropagation(...); }
//
// Everything compiles to nothing when PROFILING_ENABLED == 0.

namespace profiling {

#if PROFILING_ENABLED

enum Section : uint8_t {
  IMU_PREDICT = 0,
  RADAR_POINT,
  RADAR_FRAME,
  BARO_UPDATE,
  MAVLINK_PUBLISH,
  SUPER_LOOP,  // one full loop() iteration (mean + worst case)
  NUM_SECTIONS,
};

void init();                       // enable DWT cycle counter + non-blocking Serial.begin
void add(Section s, uint32_t t0);  // accumulate (cycles() - t0) into section s
void report(uint32_t now_ms);      // 1 Hz throttled; prints running averages to Serial

// Free-running CPU cycle counter.
inline uint32_t cycles() { return ARM_DWT_CYCCNT; }

// RAII timer: measures from construction to the end of the enclosing block.
struct Scoped {
  Section  s;
  uint32_t t0;
  explicit Scoped(Section sec) : s(sec), t0(cycles()) {}
  ~Scoped() { add(s, t0); }
};

#define PROFILE_CONCAT_(a, b) a##b
#define PROFILE_CONCAT(a, b)  PROFILE_CONCAT_(a, b)
#define PROFILE_SCOPE(sec) \
  profiling::Scoped PROFILE_CONCAT(prof_scope_, __LINE__) { profiling::sec }

#else  // PROFILING_ENABLED == 0 → no-op stubs

inline void init() {}
inline void report(uint32_t) {}
#define PROFILE_SCOPE(sec) ((void)0)

#endif  // PROFILING_ENABLED

}  // namespace profiling
