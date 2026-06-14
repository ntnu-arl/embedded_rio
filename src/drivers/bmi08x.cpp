#include "drivers/bmi08x.h"
#include <Wire.h>

static constexpr uint8_t ACC_ADDR = 0x19;
static constexpr uint8_t GYR_ADDR = 0x69;

static constexpr uint8_t REG_ACC_CHIP_ID  = 0x00;
static constexpr uint8_t REG_ACC_X_LSB    = 0x12;
static constexpr uint8_t REG_ACC_CONF     = 0x40;
static constexpr uint8_t REG_ACC_RANGE    = 0x41;
static constexpr uint8_t REG_ACC_PWR_CONF = 0x7C;
static constexpr uint8_t REG_ACC_PWR_CTRL = 0x7D;

static constexpr uint8_t REG_GYR_CHIP_ID   = 0x00;
static constexpr uint8_t REG_GYR_X_LSB     = 0x02;
static constexpr uint8_t REG_GYR_RANGE     = 0x0F;
static constexpr uint8_t REG_GYR_BANDWIDTH = 0x10;

static constexpr uint8_t CHIP_ID_ACC_BMI085 = 0x1F;
static constexpr uint8_t CHIP_ID_ACC_BMI088 = 0x1E;
static constexpr uint8_t CHIP_ID_GYR        = 0x0F;

// Range register 0x02 selects ±8 g on BMI085 and ±12 g on BMI088.
static constexpr uint8_t ACC_RANGE_REG_BMI085 = 0x02;
static constexpr float   ACC_SCALE_BMI085     = 8.0f  * 9.80665f / 32768.0f;

static constexpr uint8_t ACC_RANGE_REG_BMI088 = 0x02;
static constexpr float   ACC_SCALE_BMI088     = 12.0f * 9.80665f / 32768.0f;

static constexpr uint8_t GYR_RANGE_REG_VAL = 0x00;
static constexpr float   GYR_SCALE         = (1.0f / 16.384f) * (M_PI / 180.0f);

static bool    s_i2c_ok    = true;
static ImuType s_imu_type  = ImuType::BMI085;
static float   s_acc_scale = ACC_SCALE_BMI085;

static bool i2cWriteReg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission();
  if (err != 0) { s_i2c_ok = false; return false; }
  return true;
}

static bool i2cReadReg(uint8_t dev, uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { s_i2c_ok = false; out = 0; return false; }
  if (Wire.requestFrom(dev, (uint8_t)1) != 1) { s_i2c_ok = false; out = 0; return false; }
  out = Wire.read();
  return true;
}

static bool i2cReadBurst(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { s_i2c_ok = false; memset(buf, 0, len); return false; }
  if (Wire.requestFrom(dev, len) != len) { s_i2c_ok = false; memset(buf, 0, len); return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool bmi08xInit(ImuType type) {
  s_imu_type = type;
  uint8_t id = 0;

  const bool is088        = (type == ImuType::BMI088);
  const uint8_t acc_range = is088 ? ACC_RANGE_REG_BMI088 : ACC_RANGE_REG_BMI085;
  s_acc_scale             = is088 ? ACC_SCALE_BMI088     : ACC_SCALE_BMI085;
  const uint8_t exp_acc_id = is088 ? CHIP_ID_ACC_BMI088  : CHIP_ID_ACC_BMI085;

  if (!i2cWriteReg(ACC_ADDR, REG_ACC_PWR_CTRL, 0x04)) return false;
  delay(5);
  if (!i2cWriteReg(ACC_ADDR, REG_ACC_PWR_CONF, 0x00)) return false;
  delay(50);

  if (!i2cReadReg(ACC_ADDR, REG_ACC_CHIP_ID, id)) return false;
  Serial.print("Accel chip ID: 0x"); Serial.println(id, HEX);
  if (id != exp_acc_id) {
    Serial.print("Expected 0x"); Serial.print(exp_acc_id, HEX);
    Serial.println(" — init failed");
    return false;
  }

  if (!i2cWriteReg(ACC_ADDR, REG_ACC_RANGE, acc_range)) return false;
  delay(2);

  // 200 Hz ODR / normal bandwidth. Default is 100 Hz, which causes the
  // same accelerometer reading to be processed multiple times per poll
  // cycle and produces velocity oscillations at rest.
  if (!i2cWriteReg(ACC_ADDR, REG_ACC_CONF, 0xA9)) return false;
  delay(2);

  if (!i2cReadReg(GYR_ADDR, REG_GYR_CHIP_ID, id)) return false;
  Serial.print("Gyro  chip ID: 0x"); Serial.println(id, HEX);
  if (id != CHIP_ID_GYR) {
    Serial.println("Gyro chip ID mismatch — init failed");
    return false;
  }

  if (!i2cWriteReg(GYR_ADDR, REG_GYR_RANGE, GYR_RANGE_REG_VAL)) return false;
  delay(2);

  // 200 Hz ODR / 64 Hz BW (alternative 0x04 = 23 Hz BW for smoother but laggier).
  if (!i2cWriteReg(GYR_ADDR, REG_GYR_BANDWIDTH, 0x06)) return false;
  delay(2);

  s_i2c_ok = true;
  Serial.print("BMI08"); Serial.print(is088 ? '8' : '5');
  Serial.println(" init OK");
  return true;
}

bool bmi08xRead(rio::Vec3& acc, rio::Vec3& gyr) {
  uint8_t buf[6];

  if (!i2cReadBurst(ACC_ADDR, REG_ACC_X_LSB, buf, 6)) return false;
  acc.x() = (int16_t)(buf[0] | (buf[1] << 8)) * s_acc_scale;
  acc.y() = (int16_t)(buf[2] | (buf[3] << 8)) * s_acc_scale;
  acc.z() = (int16_t)(buf[4] | (buf[5] << 8)) * s_acc_scale;

  if (!i2cReadBurst(GYR_ADDR, REG_GYR_X_LSB, buf, 6)) return false;
  gyr.x() = (int16_t)(buf[0] | (buf[1] << 8)) * GYR_SCALE;
  gyr.y() = (int16_t)(buf[2] | (buf[3] << 8)) * GYR_SCALE;
  gyr.z() = (int16_t)(buf[4] | (buf[5] << 8)) * GYR_SCALE;

  return true;
}

void bmi08xRecover() {
  Serial.println("I2C error — recovering...");

  Wire.end();
  pinMode(19, OUTPUT);
  for (int i = 0; i < 16; i++) {
    digitalWriteFast(19, LOW);  delayMicroseconds(5);
    digitalWriteFast(19, HIGH); delayMicroseconds(5);
  }
  pinMode(19, INPUT);

  Wire.begin();
  Wire.setClock(400000);
  delay(10);

  if (bmi08xInit(s_imu_type)) Serial.println("Recovery OK");
  else                        Serial.println("Recovery FAILED");
}

void bmi08xScanI2C() {
  Serial.println("Scanning I2C bus...");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  0x"); Serial.println(a, HEX);
      n++;
    }
  }
  Serial.print("  Total: "); Serial.print(n); Serial.println(" device(s)");
  Serial.println("──────────────────────────────────");
}
