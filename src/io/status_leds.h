#pragma once

namespace leds {

void init();              // pin mode + initial LOW
void setPeriphOk(bool ok);
void setFilterOk(bool ok);
void tick();              // writes pins only on change

}  // namespace leds
