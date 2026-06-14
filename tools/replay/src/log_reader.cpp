#include "log_reader.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace replay {

namespace {

// Split a single CSV line into up to max_fields pointers. Modifies the buffer
// in place (replaces ',' with '\0'). Returns the number of fields found.
size_t splitCommas(char* line, char* fields[], size_t max_fields) {
  size_t n = 0;
  char*  p = line;
  fields[n++] = p;
  while (*p && n < max_fields) {
    if (*p == ',') {
      *p = '\0';
      fields[n++] = p + 1;
    }
    ++p;
  }
  return n;
}

float parseFloat(const char* s) { return std::strtof(s, nullptr); }

}  // namespace

LogReader::LogReader(const std::string& path) : in_(path) {
  if (!in_) {
    throw std::runtime_error("LogReader: cannot open " + path);
  }
}

bool LogReader::nextEvent(ReplayEvent& out) {
  while (true) {
    // Flush a buffered IMU/BAR event captured during radar accumulation.
    if (have_pending_ && !radar_active_) {
      out           = pending_;
      have_pending_ = false;
      return true;
    }

    if (!std::getline(in_, line_)) {
      // EOF — flush any pending radar frame, then any pending IMU/BAR event.
      if (radar_active_) {
        out.kind  = EventKind::RADAR_FRAME;
        out.t_s   = radar_t_;
        out.radar.swap(radar_buf_);
        radar_buf_.clear();
        radar_active_ = false;
        return true;
      }
      if (have_pending_) {
        out           = pending_;
        have_pending_ = false;
        return true;
      }
      return false;
    }

    if (line_.empty()) continue;

    if (!header_skipped_) {
      // Tolerate an absent header (numeric first byte → data row).
      if (line_.size() >= 1 && (line_[0] == 't' || line_[0] == 'T') &&
          line_.find("type") != std::string::npos) {
        header_skipped_ = true;
        continue;
      }
      header_skipped_ = true;  // assume no header on first non-empty line
    }

    // Buffer for in-place tokenization.
    char buf[256];
    const size_t L = std::min(line_.size(), sizeof(buf) - 1);
    std::memcpy(buf, line_.data(), L);
    buf[L] = '\0';

    char* f[10] = {nullptr};
    const size_t nf = splitCommas(buf, f, 10);
    if (nf < 2) continue;  // malformed

    const char* type = f[0];
    const float t_s  = parseFloat(f[1]);

    if (std::strcmp(type, "IMU") == 0) {
      if (nf < 8) continue;
      // If a radar frame is being accumulated, flush it first and re-buffer
      // this IMU event.
      ReplayEvent imu_ev{};
      imu_ev.kind = EventKind::IMU;
      imu_ev.t_s  = t_s;
      imu_ev.imu.t      = t_s;
      imu_ev.imu.acc.x() = parseFloat(f[2]);
      imu_ev.imu.acc.y() = parseFloat(f[3]);
      imu_ev.imu.acc.z() = parseFloat(f[4]);
      imu_ev.imu.gyr.x() = parseFloat(f[5]);
      imu_ev.imu.gyr.y() = parseFloat(f[6]);
      imu_ev.imu.gyr.z() = parseFloat(f[7]);

      if (radar_active_) {
        // Flush radar frame now; stash this IMU event for the next call.
        out.kind  = EventKind::RADAR_FRAME;
        out.t_s   = radar_t_;
        out.radar.swap(radar_buf_);
        radar_buf_.clear();
        radar_active_ = false;

        pending_      = imu_ev;
        have_pending_ = true;
        return true;
      }
      out = imu_ev;
      return true;

    } else if (std::strcmp(type, "RAD") == 0) {
      if (nf < 7) continue;
      // f[2] = point_idx (unused), f[3..5] = x,y,z, f[6] = vr.
      rio::RadarDoppler d{};
      d.t      = t_s;
      d.u_R.x() = parseFloat(f[3]);
      d.u_R.y() = parseFloat(f[4]);
      d.u_R.z() = parseFloat(f[5]);
      d.vr     = parseFloat(f[6]);
      d.sigma  = 0.0f;  // unused inside correct(); R uses params_.sigma_vr.

      if (radar_active_ && t_s != radar_t_) {
        // Flush previous frame, start a new one with this point.
        out.kind  = EventKind::RADAR_FRAME;
        out.t_s   = radar_t_;
        out.radar.swap(radar_buf_);
        radar_buf_.clear();

        radar_t_ = t_s;
        radar_buf_.push_back(d);
        radar_active_ = true;
        return true;
      }
      if (!radar_active_) {
        radar_t_      = t_s;
        radar_active_ = true;
      }
      radar_buf_.push_back(d);
      // continue reading more rows

    } else if (std::strcmp(type, "BAR") == 0) {
      if (nf < 4) continue;
      ReplayEvent baro_ev{};
      baro_ev.kind = EventKind::BARO;
      baro_ev.t_s  = t_s;
      baro_ev.baro.t           = t_s;
      baro_ev.baro.temp_c      = parseFloat(f[2]);
      baro_ev.baro.pressure_pa = parseFloat(f[3]);

      if (radar_active_) {
        out.kind  = EventKind::RADAR_FRAME;
        out.t_s   = radar_t_;
        out.radar.swap(radar_buf_);
        radar_buf_.clear();
        radar_active_ = false;

        pending_      = baro_ev;
        have_pending_ = true;
        return true;
      }
      out = baro_ev;
      return true;
    }
    // Unknown type → skip.
  }
}

}  // namespace replay
