#include "io/debug.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "filter/eskf_node.h"

namespace debug {

void init() {
#if USB_PRINT_ENABLED
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Teensy 4.x preserves fault info in RAM2 across a hard fault; dump it
  // if the previous run crashed.
  if (CrashReport) {
    Serial.println("──── PREVIOUS CRASH ────");
    Serial.print(CrashReport);
    Serial.println("────────────────────────");
  }
  Serial.println("RIO ESKF ready");
#endif
}

void printState(const rio::NominalState& x, const rio::Mat21& P) {
#if USB_PRINT_ENABLED
  Serial.print("p_WI=[");
  Serial.print(x.p_WI.x(), 6); Serial.print(", ");
  Serial.print(x.p_WI.y(), 6); Serial.print(", ");
  Serial.print(x.p_WI.z(), 6); Serial.println("]");

  Serial.print("v_WI=[");
  Serial.print(x.v_WI.x(), 6); Serial.print(", ");
  Serial.print(x.v_WI.y(), 6); Serial.print(", ");
  Serial.print(x.v_WI.z(), 6); Serial.println("]");

  Serial.print("q_WI=[");
  Serial.print(x.q_WI.w(), 6); Serial.print(", ");
  Serial.print(x.q_WI.x(), 6); Serial.print(", ");
  Serial.print(x.q_WI.y(), 6); Serial.print(", ");
  Serial.print(x.q_WI.z(), 6); Serial.println("]");

  Serial.print("b_a=[");
  Serial.print(x.b_a.x(), 6); Serial.print(", ");
  Serial.print(x.b_a.y(), 6); Serial.print(", ");
  Serial.print(x.b_a.z(), 6); Serial.println("]");

  Serial.print("b_g=[");
  Serial.print(x.b_g.x(), 6); Serial.print(", ");
  Serial.print(x.b_g.y(), 6); Serial.print(", ");
  Serial.print(x.b_g.z(), 6); Serial.println("]");

  Serial.print("p_IR=[");
  Serial.print(x.p_IR.x(), 6); Serial.print(", ");
  Serial.print(x.p_IR.y(), 6); Serial.print(", ");
  Serial.print(x.p_IR.z(), 6); Serial.println("]");

  Serial.print("q_IR=[");
  Serial.print(x.q_IR.w(), 6); Serial.print(", ");
  Serial.print(x.q_IR.x(), 6); Serial.print(", ");
  Serial.print(x.q_IR.y(), 6); Serial.print(", ");
  Serial.print(x.q_IR.z(), 6); Serial.println("]");

  // Per-group std devs: cov_xy = avg(σ_x, σ_y), cov_z = σ_z.
  // Groups: p, v, b_a, att, b_g, p_IR, q_IR.
  auto s  = [&](int i){ return sqrtf(fabsf(P(i, i))); };
  auto xy = [&](int i){ return 0.5f * (s(i) + s(i + 1)); };
  Serial.print("cov_xy=[");
  Serial.print(xy( 0), 6); Serial.print(", "); Serial.print(xy( 3), 6); Serial.print(", ");
  Serial.print(xy( 6), 6); Serial.print(", "); Serial.print(xy( 9), 6); Serial.print(", ");
  Serial.print(xy(12), 6); Serial.print(", "); Serial.print(xy(15), 6); Serial.print(", ");
  Serial.print(xy(18), 6); Serial.println("]");
  Serial.print("cov_z= [");
  Serial.print(s( 2), 6); Serial.print(", "); Serial.print(s( 5), 6); Serial.print(", ");
  Serial.print(s( 8), 6); Serial.print(", "); Serial.print(s(11), 6); Serial.print(", ");
  Serial.print(s(14), 6); Serial.print(", "); Serial.print(s(17), 6); Serial.print(", ");
  Serial.print(s(20), 6); Serial.println("]");
#else
  (void)x; (void)P;
#endif
}

void tickRates(uint32_t now_ms) {
  static uint32_t s_last_ms = 0;
  if (now_ms - s_last_ms < 1000) return;
  const float dt_s = (now_ms - s_last_ms) * 1e-3f;
  s_last_ms = now_ms;

  // Always reset counters even when USB is off, so they don't roll over.
  const auto stats = eskf_node::getAndResetStats();

#if USB_PRINT_ENABLED
  Serial.print("RATES imu_hz=");
  Serial.print(stats.imu   / dt_s, 1);
  Serial.print(" radar_hz=");
  Serial.print(stats.radar / dt_s, 1);
  Serial.print(" baro_hz=");
  Serial.println(stats.baro / dt_s, 1);

  Serial.print("BARO STATS: ");
  Serial.print(stats.baro_acc); Serial.print(" accepted, ");
  Serial.print(stats.baro_rej); Serial.print(" rejected, ");
  Serial.print(stats.baro_skp); Serial.println(" skipped");

  if (stats.baro_valid) {
    Serial.print("BARO temp=");
    Serial.print(stats.baro_temp_c, 2);
    Serial.print(" C  press=");
    Serial.print(stats.baro_press_pa, 2);
    Serial.print(" Pa (");
    Serial.print(stats.baro_press_pa / 100.0f, 2);
    Serial.println(" hPa)");
  }
#else
  (void)dt_s; (void)stats;
#endif
}

}  // namespace debug
