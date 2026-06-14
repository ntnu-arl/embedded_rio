#pragma once
// BMP388 on the shared I2C bus (Wire), alongside the BMI08x IMU.
//   VDD, VDDIO → 3.3 V; GND → GND
//   SDO → GND  (I2C addr 0x76); CSB → 3.3 V (force I2C)
//   SDA → pin 18; SCL → pin 19; 400 kHz
// Pull-ups: 4.7 kΩ from SDA and SCL to 3.3 V (shared with IMU).
#include <Arduino.h>

// Caller must already have done Wire.begin() + Wire.setClock().
bool bmp388Init();
bool bmp388Read(float& temp_c, float& press_pa);

// Bus / device recovery for Wire. Toggles SCL to release a stuck SDA,
// re-inits the bus and re-applies the BMP388 config. Called from
// bmp388Read() on failure; exposed for manual invocation if needed.
void bmp388Recover();
