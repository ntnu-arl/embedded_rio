#pragma once

#include <stdint.h>

#include <rio/rio_eskf.h>

namespace debug {

void init();         // Serial.begin(115200) + CrashReport dump
void printState(const rio::NominalState& x, const rio::Mat21& P);
void tickRates(uint32_t now_ms);   // 1 Hz throttled internally

}  // namespace debug
