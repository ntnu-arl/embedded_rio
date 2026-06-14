#pragma once
// xWR6843AOP radar on Teensy 4.1 (TTL 3.3 V).
//   CLI:  Serial4 — RX=pin 7,  TX=pin 8
//   DATA: Serial3 — RX=pin 15, TX=pin 14 (TX optional)
#include <Arduino.h>

struct RadarPoint {
  float x, y, z;  // radar frame (m), TI TLV native: x=right, y=forward, z=up
  float vr;       // signed radial velocity (m/s)
};

struct RadarFrame {
  static constexpr uint32_t MAX_POINTS = 1024;

  uint32_t   numObj;
  uint32_t   numTLV;
  uint32_t   numRaw;

  RadarPoint raw[MAX_POINTS];

  bool valid;
};

bool xwr6843aopInit();

// Drains UART FIFO non-blocking; sets frame.valid = true on a complete frame.
void xwr6843aopUpdate(RadarFrame& frame);

void xwr6843aopDrainCli();

void xwr6843aopPrintRaw(const RadarFrame& frame);
