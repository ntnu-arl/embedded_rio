#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <rio/rio_eskf.h>

namespace replay {

enum class EventKind { IMU, RADAR_FRAME, BARO };

struct ReplayEvent {
  EventKind kind;
  float     t_s;
  rio::ImuSample                imu;
  std::vector<rio::RadarDoppler> radar;  // populated for RADAR_FRAME
  rio::BarometerSample          baro;
};

// Streaming reader for the multiplexed CSV produced by sd_logger.cpp.
// Schema (variable-width by leading "type" column):
//   IMU,t_s,ax,ay,az,gx,gy,gz
//   RAD,t_s,point_idx,x,y,z,vr           (one CSV row per radar point)
//   BAR,t_s,temp_c,press_pa
//
// Consecutive RAD rows sharing the same t_s are grouped into a single
// RADAR_FRAME event. The grouping flushes when a RAD row with a new t_s
// arrives, when any non-RAD row arrives, or at EOF.
class LogReader {
public:
  explicit LogReader(const std::string& path);

  // Returns false at EOF after all buffered radar points are flushed.
  // On true, `out` is fully populated.
  bool nextEvent(ReplayEvent& out);

private:
  std::ifstream in_;
  bool          header_skipped_{false};
  std::string   line_;

  // One-event lookahead for IMU/BAR while we accumulate radar rows.
  bool          have_pending_{false};
  ReplayEvent   pending_{};

  // Radar accumulator.
  bool                           radar_active_{false};
  float                          radar_t_{0.0f};
  std::vector<rio::RadarDoppler> radar_buf_;
};

}  // namespace replay
