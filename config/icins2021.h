#pragma once
// ──────────────────────────────────────────────────────────────
// Radar chirp configuration — ICINS 2021
//
// C.Doer, G.F.Trommer: Yaw aided Radar Inertial Odometry
// using Manhattan World Assumptions; ICINS2021
//
// d_max  = 20.00 m,   d_res  = 0.07 m
// v_max  =  4.00 m/s, v_res  = 0.13 m/s
// N_range = 256, N_frames = 60, N_velocity_fft = 64
// T_f = 18.39 ms, T_c = 102.19 us, B = 2.23 GHz
// ──────────────────────────────────────────────────────────────

static const char* const RADAR_CFG_ICINS2021[] = {
  "sensorStop",
  "configDataPort 921600 1",  // keep: sets DATA UART to 921600 baud
  "sensorStop",
  "flushCfg",
  "dfeDataOutputMode 1",
  "channelCfg 15 7 0",
  "adcCfg 2 1",
  "adcbufCfg -1 0 1 1 1",
  "profileCfg 0 60.00 38.44 7.00 63.75 0 0 35.04 1.00 255 4675.80 0 0 30",
  "chirpCfg 0 0 0 0 0 0 0 1",
  "chirpCfg 1 1 0 0 0 0 0 2",
  "chirpCfg 2 2 0 0 0 0 0 4",
  "frameCfg 0 2 60 0 100.0 2 0.00",
  "lowPower 0 0",
  "guiMonitor -1 1 0 0 0 0 1",
  "cfarCfg -1 0 2 8 4 3 0 8 0",
  "cfarCfg -1 1 0 4 2 3 1 20 1",
  "multiObjBeamForming -1 1 0.5",
  "clutterRemoval -1 0",
  "aoaFovCfg -1 -90 90 -90 90",
  "cfarFovCfg -1 0 0.1 19.0",
  "cfarFovCfg -1 1 -8.00 8.00",
  "calibDcRangeSig -1 0 -5 8 256",
  "extendedMaxVelocity -1 0",
  "lvdsStreamCfg -1 0 0 0",
  "measureRangeBiasAndRxChanPhase 0 1.5 0.2",
  "compRangeBiasAndRxChanPhase 0.0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0",
  "CQRxSatMonitor 0 3 5 103 0",
  "CQSigImgMonitor 0 95 6",
  "analogMonitor 0 0",
  "calibData 0 0 0",
  "sensorStart",
};
static constexpr size_t NUM_CFG_ICINS2021 = sizeof(RADAR_CFG_ICINS2021) / sizeof(RADAR_CFG_ICINS2021[0]);
