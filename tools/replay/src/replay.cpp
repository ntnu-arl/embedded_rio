// Offline replay of SD-card sensor logs through rio::RioEskf.
//
// Edit makeParams() / P0_diag below to retune sigmas and initial covariance,
// then rebuild and run:
//
//   cmake -S tools/replay -B build/replay -DCMAKE_BUILD_TYPE=Release
//   cmake --build build/replay -j
//   ./build/replay/rio_replay path/to/LOG0001.CSV out_dir/
//
// Outputs (in out_dir/):
//   state.csv         — t,source,p_WI,v_WI,q_WI,b_a,b_g,p_IR,q_IR
//   cov_diag.csv      — t,source,P00..P20
//   radar_innov.csv   — t,point_idx,status,residual,S,norm  (long format)
//   baro_innov.csv    — t,initialized,accepted,rejected,skipped,dz_meas,
//                       dz_pred,residual,S,norm

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <rio/rio_eskf.h>
#include <rio/measurements/radar_doppler.h>
#include <rio/measurements/barometer.h>

#include "config.h"      // firmware src/config.h — single source of truth
                         // for BARO_AIDING_ENABLED, BARO_AIDING_DIFFERENTIAL,
                         // RADAR_AIDING_ENABLED, etc.
#include "log_reader.h"

namespace fs = std::filesystem;

// ── Tunable parameters ────────────────────────────────────────────────────────
// Mirrors src/main.cpp verbatim. Edit these to try alternate tunings.

static rio::Params makeParams() {
  rio::Params p;

  p.g_W = rio::Vec3(0.0f, 0.0f, -9.80665f);

  p.sigma_acc = 1.86e-3f;
  p.sigma_ba  = 1.86e-4f;

  p.sigma_gyr = 2.443461e-4f;
  p.sigma_bg  = 2.443461e-5f;

  p.tau_ba = 700.0f;
  p.tau_bg = 450.0f;

  p.min_dt = 1e-6f;
  p.max_dt = 0.1f;

  p.q_IR = rio::Quat(0.5f, -0.5f, -0.5f, 0.5f);  // Eigen (w,x,y,z)
  p.p_IR = rio::Vec3(0.01f, 0.03f, -0.01f);

  return p;
}

static constexpr rio::RadarDopplerMeasurement::Params kRadarParams{
    /*sigma_vr=*/0.038f, /*vr_sign=*/1.0f,
    /*gate_nsigma=*/3.0f, /*gating=*/true,
    /*underweight=*/RADAR_UNDERWEIGHTING_ENABLED != 0};

static constexpr rio::BarometerMeasurement::Params kBaroParams{
    /*sigma_dz=*/0.20f, /*z_sign=*/1.0f,
    /*gate_nsigma=*/3.0f, /*gating=*/true};

static constexpr float P0_diag[21] = {
  1e-6f  , 1e-6f  , 1e-6f  , // ego position (m)
  1e-6f  , 1e-6f  , 1e-6f  , // ego velocity (m/s)
  1e-4f  , 1e-4f  , 1e-4f  , // accelerometer bias (m/s²)
  1e-2f  , 1e-2f  , 1e-2f  , // ego attitude (rad): roll/pitch from gravity, yaw unknown
  1e-5f  , 1e-5f  , 1e-5f  , // gyroscope bias (rad/s)
  2.0e-3f, 2.0e-3f, 2.0e-3f, // radar position relative to IMU (m)
  1.0e-4f, 1.0e-4f, 1.0e-4f, // radar attitude relative to IMU (rad)
};

// ── Output writers ────────────────────────────────────────────────────────────

namespace {

struct Outputs {
  std::FILE* state{};
  std::FILE* cov{};
  std::FILE* radar{};
  std::FILE* baro{};
};

void writeStateHeader(std::FILE* f) {
  std::fprintf(f,
    "t,source,"
    "p_x,p_y,p_z,v_x,v_y,v_z,"
    "q_w,q_x,q_y,q_z,"
    "ba_x,ba_y,ba_z,bg_x,bg_y,bg_z,"
    "p_IR_x,p_IR_y,p_IR_z,q_IR_w,q_IR_x,q_IR_y,q_IR_z\n");
}

void writeCovHeader(std::FILE* f) {
  std::fprintf(f, "t,source");
  for (int i = 0; i < 21; ++i) std::fprintf(f, ",P%02d", i);
  std::fprintf(f, "\n");
}

void writeRadarHeader(std::FILE* f) {
  // B is the second-order underweighting term added to S (S = H P H^T + R + B).
  // 0 when RADAR_UNDERWEIGHTING_ENABLED is off; the column is still emitted so
  // downstream tooling has a stable schema.
  std::fprintf(f, "t,point_idx,status,residual,S,norm,B\n");
}

void writeBaroHeader(std::FILE* f) {
  std::fprintf(f,
    "t,initialized,accepted,rejected,skipped,"
    "dz_meas,dz_pred,residual,S,norm\n");
}

void writeStateRow(std::FILE* f, float t, const char* src,
                   const rio::NominalState& x) {
  std::fprintf(f,
    "%.6f,%s,"
    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
    "%.6f,%.6f,%.6f,%.6f,"
    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
    t, src,
    x.p_WI.x(), x.p_WI.y(), x.p_WI.z(),
    x.v_WI.x(), x.v_WI.y(), x.v_WI.z(),
    x.q_WI.w(), x.q_WI.x(), x.q_WI.y(), x.q_WI.z(),
    x.b_a.x(),  x.b_a.y(),  x.b_a.z(),
    x.b_g.x(),  x.b_g.y(),  x.b_g.z(),
    x.p_IR.x(), x.p_IR.y(), x.p_IR.z(),
    x.q_IR.w(), x.q_IR.x(), x.q_IR.y(), x.q_IR.z());
}

void writeCovRow(std::FILE* f, float t, const char* src, const rio::Mat21& P) {
  std::fprintf(f, "%.6f,%s", t, src);
  for (int i = 0; i < 21; ++i) std::fprintf(f, ",%.9g", P(i, i));
  std::fprintf(f, "\n");
}

// Map MeasurementUpdate::Status to the legacy 0/1/2 status codes used by the
// radar_innov.csv consumers (0=accepted, 1=rejected, 2=skipped).
unsigned legacyStatus(rio::MeasurementUpdate::Status s) {
  switch (s) {
    case rio::MeasurementUpdate::Accepted: return 0;
    case rio::MeasurementUpdate::Rejected: return 1;
    default:                          return 2;  // Skipped or NotReady
  }
}

void writeRadarInnov(std::FILE* f, float t,
                     const std::vector<rio::MeasurementUpdate>& updates) {
  for (size_t i = 0; i < updates.size(); ++i) {
    const float r = updates[i].residual;
    const float S = updates[i].S;
    const float B = updates[i].B;
    const float norm = (S > 0.0f && std::isfinite(r) && std::isfinite(S))
                           ? std::fabs(r) / std::sqrt(S)
                           : NAN;
    std::fprintf(f, "%.6f,%zu,%u,%.6g,%.6g,%.6g,%.6g\n",
                 t, i, legacyStatus(updates[i].status), r, S, norm, B);
  }
}

void writeBaroInnov(std::FILE* f, float t,
                    const rio::MeasurementUpdate& u,
                    const rio::BarometerMeasurement& m) {
  const float r    = u.residual;
  const float S    = u.S;
  const float norm = (S > 0.0f && std::isfinite(r) && std::isfinite(S))
                         ? std::fabs(r) / std::sqrt(S)
                         : NAN;
  const bool initialized = m.justInitialized();
  const bool accepted    = (u.status == rio::MeasurementUpdate::Accepted);
  const bool rejected    = (u.status == rio::MeasurementUpdate::Rejected);
  const bool skipped     = (u.status == rio::MeasurementUpdate::Skipped);
  // Match the old CSV: when only the anchor was initialized, dz_meas /
  // dz_pred / residual / S are all reported as zero.
  const float dz_meas  = initialized ? 0.f : m.lastDzMeas();
  const float dz_pred  = initialized ? 0.f : m.lastDzPred();
  const float r_out    = initialized ? 0.f : r;
  const float S_out    = initialized ? 0.f : S;
  const float norm_out = initialized ? NAN : norm;
  std::fprintf(f, "%.6f,%d,%d,%d,%d,%.6g,%.6g,%.6g,%.6g,%.6g\n",
               t,
               initialized ? 1 : 0,
               accepted    ? 1 : 0,
               rejected    ? 1 : 0,
               skipped     ? 1 : 0,
               dz_meas, dz_pred, r_out, S_out, norm_out);
}

}  // namespace

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  // Aiding mode comes from src/config.h (shared with firmware). Flip the
  // #defines there and rebuild to switch behavior — no CLI flags.
  constexpr bool radar_enabled = (RADAR_AIDING_ENABLED != 0);
  constexpr bool baro_enabled  = (BARO_AIDING_ENABLED  != 0);
  constexpr bool baro_absolute = (BARO_AIDING_DIFFERENTIAL == 0);

  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <log.csv> <out_dir>\n", argv[0]);
    return 2;
  }
  const std::string log_path = argv[1];
  const fs::path    out_dir  = argv[2];

  std::error_code ec;
  fs::create_directories(out_dir, ec);
  if (ec) {
    std::fprintf(stderr, "cannot create %s: %s\n",
                 out_dir.string().c_str(), ec.message().c_str());
    return 2;
  }

  Outputs out;
  out.state = std::fopen((out_dir / "state.csv").string().c_str(), "w");
  out.cov   = std::fopen((out_dir / "cov_diag.csv").string().c_str(), "w");
  out.radar = std::fopen((out_dir / "radar_innov.csv").string().c_str(), "w");
  out.baro  = std::fopen((out_dir / "baro_innov.csv").string().c_str(), "w");
  if (!out.state || !out.cov || !out.radar || !out.baro) {
    std::fprintf(stderr, "failed to open output files in %s\n",
                 out_dir.string().c_str());
    return 2;
  }
  writeStateHeader(out.state);
  writeCovHeader(out.cov);
  writeRadarHeader(out.radar);
  writeBaroHeader(out.baro);

  rio::RioEskf  eskf;
  rio::Params   params = makeParams();
  eskf.setParams(params);

  rio::NominalState x0;
  x0.p_IR = params.p_IR;
  x0.q_IR = params.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);

  // Persistent measurement object — holds the baro anchor across calls.
  // In differential mode the anchor moves on every accept; in absolute mode
  // it stays at the boot-time reference.
  rio::BarometerMeasurement::Params baro_params = kBaroParams;
  baro_params.reset_anchor_on_accept = !baro_absolute;
  rio::BarometerMeasurement baro_meas(baro_params);

  bool             time_init = false;
  bool             att_init  = false;
  float            last_t    = 0.0f;
  rio::ImuSample   last_imu{};

  // Counters for end-of-run summary.
  size_t n_imu = 0, n_radar_frames = 0, n_radar_pts = 0, n_baro = 0;
  size_t n_radar_accepted = 0, n_radar_rejected = 0, n_radar_skipped = 0;
  size_t n_baro_accepted = 0, n_baro_rejected = 0, n_baro_skipped = 0;

  replay::LogReader reader(log_path);
  replay::ReplayEvent ev;

  while (reader.nextEvent(ev)) {
    switch (ev.kind) {
      case replay::EventKind::IMU: {
        ++n_imu;

        if (!time_init) {
          last_t    = ev.t_s;
          time_init = true;
          break;
        }
        if (!att_init) {
          if (eskf.initAttitudeFromGravity(ev.imu.acc, P0_diag, ev.t_s)) {
            att_init = true;
            last_t   = ev.t_s;
            last_imu = ev.imu;
            writeStateRow(out.state, ev.t_s, "INIT", eskf.getState());
            writeCovRow  (out.cov,   ev.t_s, "INIT", eskf.getCovariance());
          }
          break;
        }

        const float dt = ev.t_s - last_t;
        last_t   = ev.t_s;
        last_imu = ev.imu;

        if (dt >= params.min_dt && dt <= params.max_dt) {
          eskf.predict       (ev.imu, dt);
          eskf.insPropagation(ev.imu, dt);
        }
        writeStateRow(out.state, ev.t_s, "IMU", eskf.getState());
        writeCovRow  (out.cov,   ev.t_s, "IMU", eskf.getCovariance());
        break;
      }

      case replay::EventKind::RADAR_FRAME: {
        ++n_radar_frames;
        n_radar_pts += ev.radar.size();
        if (!att_init || ev.radar.empty() || !radar_enabled) break;

        rio::MeasurementContext ctx{&last_imu};
        std::vector<rio::MeasurementUpdate> updates;
        updates.reserve(ev.radar.size());
        size_t n_acc = 0, n_rej = 0, n_skp = 0;
        for (const auto& d : ev.radar) {
          rio::RadarDopplerMeasurement m(kRadarParams, d.u_R, d.vr);
          const rio::MeasurementUpdate u = eskf.correct(m, ctx);
          updates.push_back(u);
          switch (u.status) {
            case rio::MeasurementUpdate::Accepted: ++n_acc; break;
            case rio::MeasurementUpdate::Rejected: ++n_rej; break;
            default:                          ++n_skp; break;
          }
        }
        // Match old batch behavior: snap P_hat_ to the predicted prior
        // when no point was accepted, so getCovariance() reflects predict.
        if (n_acc == 0) eskf.advancePriorToPosterior();

        n_radar_accepted += n_acc;
        n_radar_rejected += n_rej;
        n_radar_skipped  += n_skp;

        writeRadarInnov(out.radar, ev.t_s, updates);
        writeStateRow  (out.state, ev.t_s, "RAD",  eskf.getState());
        writeCovRow    (out.cov,   ev.t_s, "RAD",  eskf.getCovariance());
        break;
      }

      case replay::EventKind::BARO: {
        ++n_baro;
        if (!att_init || !baro_enabled) break;

        baro_meas.setSample(ev.baro);
        const rio::MeasurementUpdate u = eskf.correct(baro_meas);
        if (u.status == rio::MeasurementUpdate::Accepted) ++n_baro_accepted;
        if (u.status == rio::MeasurementUpdate::Rejected) ++n_baro_rejected;
        if (u.status == rio::MeasurementUpdate::Skipped)  ++n_baro_skipped;

        writeBaroInnov(out.baro,  ev.t_s, u, baro_meas);
        writeStateRow (out.state, ev.t_s, "BAR", eskf.getState());
        writeCovRow   (out.cov,   ev.t_s, "BAR", eskf.getCovariance());
        break;
      }
    }
  }

  std::fclose(out.state);
  std::fclose(out.cov);
  std::fclose(out.radar);
  std::fclose(out.baro);

  const char* radar_mode_tag =
      !radar_enabled ? " (aiding disabled)" :
      (RADAR_UNDERWEIGHTING_ENABLED != 0 ? " (mode: 2nd-order underweighting)" : "");
  const char* baro_mode_tag =
      !baro_enabled ? " (aiding disabled)" :
      (baro_absolute ? " (mode: absolute)" : " (mode: differential)");
  std::fprintf(stderr,
    "replay complete: imu=%zu radar_frames=%zu radar_pts=%zu baro=%zu\n"
    "  radar: %zu accepted, %zu rejected, %zu skipped%s\n"
    "  baro:  %zu accepted, %zu rejected, %zu skipped%s\n",
    n_imu, n_radar_frames, n_radar_pts, n_baro,
    n_radar_accepted, n_radar_rejected, n_radar_skipped, radar_mode_tag,
    n_baro_accepted,  n_baro_rejected,  n_baro_skipped,  baro_mode_tag);
  return 0;
}
