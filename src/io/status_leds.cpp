#include "io/status_leds.h"

#include <Arduino.h>

namespace leds {
namespace {

// Pin 2: peripherals OK (IMU + baro reading without errors).
// Pin 3: filter stable (attitude initialised from gravity).
constexpr uint8_t LED_PERIPH_PIN = 2;
constexpr uint8_t LED_FILTER_PIN = 3;

bool s_periph_ok = false;
bool s_filter_ok = false;
int  s_periph_written = -1;
int  s_filter_written = -1;

}  // namespace

void init() {
  pinMode(LED_PERIPH_PIN, OUTPUT);
  pinMode(LED_FILTER_PIN, OUTPUT);
  digitalWrite(LED_PERIPH_PIN, LOW);
  digitalWrite(LED_FILTER_PIN, LOW);
}

void setPeriphOk(bool ok) { s_periph_ok = ok; }
void setFilterOk(bool ok) { s_filter_ok = ok; }

void tick() {
  const int p = s_periph_ok ? 1 : 0;
  const int f = s_filter_ok ? 1 : 0;
  if (p != s_periph_written) {
    digitalWrite(LED_PERIPH_PIN, p ? HIGH : LOW);
    s_periph_written = p;
  }
  if (f != s_filter_written) {
    digitalWrite(LED_FILTER_PIN, f ? HIGH : LOW);
    s_filter_written = f;
  }
}

}  // namespace leds
