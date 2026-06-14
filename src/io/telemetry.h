#pragma once

#include <rio/rio_eskf.h>

namespace telemetry {

void init();   // Serial2.begin(921600)

void sendOdometry(const rio::NominalState& x, const rio::Mat21& P,
                  float t_sec, int8_t quality);

}  // namespace telemetry
