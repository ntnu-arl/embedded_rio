#include "filter/eskf_node.h"

#include <Arduino.h>

#include <rio/measurements/radar_doppler.h>
#include <rio/measurements/barometer.h>

#include "config.h"
#include "io/debug.h"
#include "io/profiling.h"
#include "io/telemetry.h"
#include "io/sd_logger.h"

namespace eskf_node {
namespace {

rio::RioEskf eskf;
rio::Params  g_params;

rio::Params makeParams() {
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

const rio::RadarDopplerMeasurement::Params g_radar_p{
    /*sigma_vr=*/0.038f, /*vr_sign=*/1.0f,
    /*gate_nsigma=*/3.0f, /*gating=*/true,
    /*underweight=*/RADAR_UNDERWEIGHTING_ENABLED != 0};

rio::BarometerMeasurement g_baro_meas(
    rio::BarometerMeasurement::Params{
        /*sigma_dz=*/0.20f, /*z_sign=*/1.0f,
        /*gate_nsigma=*/3.0f, /*gating=*/true,
        /*reset_anchor_on_accept=*/BARO_AIDING_DIFFERENTIAL != 0});

constexpr float P0_diag[21] = {
  1e-6f  , 1e-6f  , 1e-6f  , // ego position (m)
  1e-6f  , 1e-6f  , 1e-6f  , // ego velocity (m/s)
  1e-4f  , 1e-4f  , 1e-4f  , // accelerometer bias (m/s²)
  1e-2f  , 1e-2f  , 1e-2f  , // ego attitude (rad): roll/pitch from gravity, yaw unknown
  1e-5f  , 1e-5f  , 1e-5f  , // gyroscope bias (rad/s)
  2.0e-3f, 2.0e-3f, 2.0e-3f, // radar position relative to IMU (m)
  1.0e-4f, 1.0e-4f, 1.0e-4f, // radar attitude relative to IMU (rad)
};

bool           s_att_init  = false;
bool           s_time_init = false;
float          s_last_t    = 0.0f;
rio::ImuSample s_last_imu{};

// Rate counters — reset by getAndResetStats() at 1 Hz.
uint32_t s_imu_count   = 0;
uint32_t s_radar_count = 0;
uint32_t s_baro_count  = 0;
uint32_t s_baro_acc    = 0;
uint32_t s_baro_rej    = 0;
uint32_t s_baro_skp    = 0;

// Cached barometer reading for the 1 Hz logger (no extra I²C transaction).
float s_baro_temp_c   = NAN;
float s_baro_press_pa = NAN;
bool  s_baro_valid    = false;

void publishState(float t, int8_t quality) {
  const auto& x = eskf.getState();
  const auto& P = eskf.getCovariance();
#if USB_PRINT_ENABLED
  debug::printState(x, P);
#else
  telemetry::sendOdometry(x, P, t, quality);
#endif
}

}  // namespace

void init() {
  g_params = makeParams();
  eskf.setParams(g_params);

  rio::NominalState x0;
  x0.p_IR = g_params.p_IR;
  x0.q_IR = g_params.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);
}

bool isAttitudeInitialized() { return s_att_init; }

void onImu(const rio::Vec3& acc, const rio::Vec3& gyr, float t) {
#if SD_LOG_ENABLED
  sdLoggerLogImu(t, acc, gyr);
#endif

  if (!s_time_init) {
    s_last_t    = t;
    s_time_init = true;
    return;
  }

  if (!s_att_init) {
    if (!eskf.initAttitudeFromGravity(acc, P0_diag, t)) return;
    s_att_init = true;
    s_last_t   = t;
#if USB_PRINT_ENABLED
    Serial.println("Attitude initialized from gravity");
#endif
    return;
  }

  const float dt = t - s_last_t;
  s_last_t = t;

  rio::ImuSample s;
  s.t   = t;
  s.acc = acc;
  s.gyr = gyr;
  s_last_imu = s;

  if (dt >= g_params.min_dt && dt <= g_params.max_dt) {
    {
      PROFILE_SCOPE(IMU_PREDICT);
      eskf.predict(s, dt);
      eskf.insPropagation(s, dt);
    }
#if !RADAR_AIDING_ENABLED && !BARO_AIDING_ENABLED
    // No corrections will run — mirror prior into posterior so
    // getCovariance() reflects growing IMU-only uncertainty, and emit
    // the state to USB at ~50 Hz for the serial plotter.
    eskf.advancePriorToPosterior();
  #if USB_PRINT_ENABLED
    static uint32_t s_pub_last_us = 0;
    const uint32_t  now_pub_us    = micros();
    if (now_pub_us - s_pub_last_us >= 20000UL) {
      s_pub_last_us = now_pub_us;
      debug::printState(eskf.getState(), eskf.getCovariance());
    }
  #endif
#endif
  }

  s_imu_count++;
}

void onRadarFrame(const RadarFrame& frame) {
  if (!frame.valid || frame.numRaw == 0 || !s_att_init) return;

  const float t_r = static_cast<float>(millis()) * 1e-3f;

#if SD_LOG_ENABLED
  sdLoggerLogRadar(t_r, frame);
#endif

  s_radar_count++;

#if RADAR_AIDING_ENABLED
  rio::MeasurementContext ctx{&s_last_imu};
  uint32_t n_acc = 0, n_rej = 0, n_skp = 0;
  {
    PROFILE_SCOPE(RADAR_FRAME);
    for (uint32_t i = 0; i < frame.numRaw; i++) {
      rio::RadarDopplerMeasurement m(
          g_radar_p,
          rio::Vec3(frame.raw[i].x, frame.raw[i].y, frame.raw[i].z),
          frame.raw[i].vr);
      rio::MeasurementUpdate u;
      {
        PROFILE_SCOPE(RADAR_POINT);
        u = eskf.correct(m, ctx);
      }
      switch (u.status) {
        case rio::MeasurementUpdate::Accepted: ++n_acc; break;
        case rio::MeasurementUpdate::Rejected: ++n_rej; break;
        default:                               ++n_skp; break;
      }
    }
    if (n_acc == 0) eskf.advancePriorToPosterior();
  }

  const uint32_t total   = n_acc + n_rej + n_skp;
  const int8_t   quality = (total > 0)
      ? static_cast<int8_t>(n_acc * 100u / total)
      : 0;

#if USB_PRINT_ENABLED
  if (n_rej > 0 || n_skp > 0) {
    Serial.print("ESKF correct: ");
    Serial.print(n_acc); Serial.print(" accepted, ");
    Serial.print(n_rej); Serial.print(" rejected, ");
    Serial.print(n_skp); Serial.println(" skipped");
  }
#endif

  publishState(t_r, quality);
#else
  (void)t_r;
#endif
}

void onBaroSample(float temp_c, float press_pa, float t) {
  s_baro_temp_c   = temp_c;
  s_baro_press_pa = press_pa;
  s_baro_valid    = true;

#if SD_LOG_ENABLED
  sdLoggerLogBaro(t, temp_c, press_pa);
#endif

  s_baro_count++;

  if (!s_att_init) return;

#if BARO_AIDING_ENABLED
  rio::BarometerSample baro;
  baro.t           = t;
  baro.pressure_pa = press_pa;
  baro.temp_c      = temp_c;

  g_baro_meas.setSample(baro);
  rio::MeasurementUpdate u;
  {
    PROFILE_SCOPE(BARO_UPDATE);
    u = eskf.correct(g_baro_meas);
  }

#if USB_PRINT_ENABLED
  if (u.status == rio::MeasurementUpdate::Rejected) {
    Serial.print("BARO rejected: dz_meas=");
    Serial.print(g_baro_meas.lastDzMeas(), 3);
    Serial.print(" dz_pred=");
    Serial.print(g_baro_meas.lastDzPred(), 3);
    Serial.print(" residual=");
    Serial.println(u.residual, 3);
  }
#endif

  switch (u.status) {
    case rio::MeasurementUpdate::Accepted: ++s_baro_acc; break;
    case rio::MeasurementUpdate::Rejected: ++s_baro_rej; break;
    case rio::MeasurementUpdate::Skipped:  ++s_baro_skp; break;
    default: break;   // NotReady (anchor just initialized) — don't count
  }

  if (u.status == rio::MeasurementUpdate::Accepted) {
    publishState(baro.t, /*quality=*/100);
  }
#else
  (void)t;
#endif
}

void onBaroReadFailed() {
  // Differential anchor no longer matches whatever pressure the chip
  // will report after recovery — drop it.
  g_baro_meas.resetAnchor();
}

Stats getAndResetStats() {
  Stats s;
  s.imu           = s_imu_count;
  s.radar         = s_radar_count;
  s.baro          = s_baro_count;
  s.baro_acc      = s_baro_acc;
  s.baro_rej      = s_baro_rej;
  s.baro_skp      = s_baro_skp;
  s.baro_temp_c   = s_baro_temp_c;
  s.baro_press_pa = s_baro_press_pa;
  s.baro_valid    = s_baro_valid;

  s_imu_count   = 0;
  s_radar_count = 0;
  s_baro_count  = 0;
  s_baro_acc    = 0;
  s_baro_rej    = 0;
  s_baro_skp    = 0;

  return s;
}

}  // namespace eskf_node
