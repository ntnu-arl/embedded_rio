#pragma once
// BMI08x IMU on I2C (Wire), SDO1/SDO2 → VDDIO.
//   Accel: 0x19   Gyro: 0x69
//   SDA: pin 18   SCL: pin 19   400 kHz
#include <Arduino.h>
#undef B0
#undef B1
#undef B2
#undef B3
#include <rio/rio_types.h>

enum class ImuType { BMI085, BMI088 };

bool bmi08xInit(ImuType type);

// acc in m/s², gyr in rad/s. Returns false on I2C error; caller should
// invoke bmi08xRecover() to toggle SCL and re-init.
bool bmi08xRead(rio::Vec3& acc, rio::Vec3& gyr);

void bmi08xRecover();

void bmi08xScanI2C();
