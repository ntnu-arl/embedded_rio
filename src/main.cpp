#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "drivers/bmi08x.h"
#include "drivers/bmp388.h"
#include "drivers/xwr6843aop.h"
#include "io/sd_logger.h"

#include "filter/eskf_node.h"
#include "io/telemetry.h"
#include "io/debug.h"
#include "io/profiling.h"
#include "io/status_leds.h"

static constexpr ImuType IMU_TYPE = ImuType::BMI088;

static constexpr uint32_t IMU_HZ        = 200;
static constexpr uint32_t IMU_PERIOD_US = 1000000UL / IMU_HZ;
static uint32_t           s_imu_last_us = 0;

static constexpr uint32_t BARO_HZ        = 50;
static constexpr uint32_t BARO_PERIOD_US = 1000000UL / BARO_HZ;
static uint32_t           s_baro_last_us = 0;

static RadarFrame s_radar_frame;

static void sensorsInit() {
  Wire.begin();
  Wire.setClock(400000);
  bmi08xScanI2C();
  if (!bmi08xInit(IMU_TYPE)) {
    while (1) delay(100);
  }
  if (!bmp388Init()) {
    // warn only — baro is optional
  }
  delay(200);
  if (!xwr6843aopInit()) {
    while (1) delay(100);
  }
}

void setup() {
  leds::init();
  debug::init();
  profiling::init();
  telemetry::init();
  sensorsInit();
  eskf_node::init();
#if SD_LOG_ENABLED
  sdLoggerInit();
#endif
}

void loop() {
  // 1 Hz profiling report first, OUTSIDE the SUPER_LOOP scope below: the USB
  // print is profiler overhead (gone when PROFILING_ENABLED == 0) and would
  // otherwise pollute the super loop's worst case once per second.
  profiling::report(millis());

  // Mean + worst-case execution time of one full loop() iteration, covering
  // everything below including the early-return IMU paths and the SD flush.
  PROFILE_SCOPE(SUPER_LOOP);

  // IMU — read every iteration, rate-gate the ESKF dispatch.
  rio::Vec3 acc, gyr;
  if (!bmi08xRead(acc, gyr)) {
    bmi08xRecover();
    leds::setPeriphOk(false);
    leds::tick();
    return;
  }
  leds::setPeriphOk(true);
  if (!acc.allFinite() || !gyr.allFinite()) return;

  const uint32_t now_us = micros();
  if (now_us - s_imu_last_us >= IMU_PERIOD_US) {
    s_imu_last_us = now_us;
    eskf_node::onImu(acc, gyr, millis() * 1e-3f);
  }

  // Radar — drain UART, dispatch when a frame is complete.
  xwr6843aopDrainCli();
  xwr6843aopUpdate(s_radar_frame);
  if (s_radar_frame.valid) {
    eskf_node::onRadarFrame(s_radar_frame);
  }

  // Baro — rate-gated read + dispatch.
  if (now_us - s_baro_last_us >= BARO_PERIOD_US) {
    s_baro_last_us = now_us;
    float temp_c, press_pa;
    if (bmp388Read(temp_c, press_pa)) {
      eskf_node::onBaroSample(temp_c, press_pa, millis() * 1e-3f);
    } else {
      eskf_node::onBaroReadFailed();
    }
  }

  // Status + diagnostics.
  leds::setFilterOk(eskf_node::isAttitudeInitialized());
  leds::tick();
  debug::tickRates(millis());

#if SD_LOG_ENABLED
  static uint32_t s_flush_last_ms = 0;
  const uint32_t  now_ms          = millis();
  if (now_ms - s_flush_last_ms >= SD_LOG_FLUSH_INTERVAL_S * 1000UL) {
    s_flush_last_ms = now_ms;
    sdLoggerFlush();
  }
#endif
}
