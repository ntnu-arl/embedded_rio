#include "drivers/bmp388.h"
#include <Wire.h>

#include "config.h"

// SDO → GND → 0x76. SDO → VDDIO → 0x77.
static constexpr uint8_t BMP388_I2C_ADDR = 0x76;

// Shared with the BMI08x; both run at 400 kHz on Wire.
static constexpr uint32_t BMP388_I2C_CLOCK_HZ = 400000;
static constexpr uint8_t  BMP388_SCL_PIN      = 19;

// ── Register map (BMP388 datasheet §4.2) ────────────────────────────────────
static constexpr uint8_t REG_CHIP_ID     = 0x00;
static constexpr uint8_t REG_ERR         = 0x02;
static constexpr uint8_t REG_STATUS      = 0x03;
static constexpr uint8_t REG_DATA_0      = 0x04;  // press[7:0]
static constexpr uint8_t REG_NVM_PAR     = 0x31;  // 21 bytes of trim
static constexpr uint8_t REG_PWR_CTRL    = 0x1B;
static constexpr uint8_t REG_OSR         = 0x1C;
static constexpr uint8_t REG_ODR         = 0x1D;
static constexpr uint8_t REG_CONFIG      = 0x1F;
static constexpr uint8_t REG_CMD         = 0x7E;

static constexpr uint8_t CHIP_ID_BMP388  = 0x50;
static constexpr uint8_t CMD_SOFT_RESET  = 0xB6;

// PWR_CTRL: press_en=1, temp_en=1, mode=normal (0b11).
static constexpr uint8_t PWR_CTRL_VAL    = 0x33;
// OSR: osr_p = 8x (0b011), osr_t = 1x (0b000).
static constexpr uint8_t OSR_VAL         = 0x03;
// ODR: 50 Hz selector = 0x02.
static constexpr uint8_t ODR_VAL         = 0x02;
// CONFIG: IIR filter coeff 3 (selector 0b010) in bits[3:1].
static constexpr uint8_t CONFIG_VAL      = 0x04;

// ── Float trim coefficients (BMP3 reference compensation) ───────────────────
struct QuantTrim {
  float par_t1;
  float par_t2;
  float par_t3;
  float par_p1;
  float par_p2;
  float par_p3;
  float par_p4;
  float par_p5;
  float par_p6;
  float par_p7;
  float par_p8;
  float par_p9;
  float par_p10;
  float par_p11;
  float t_lin;
};

static QuantTrim s_q{};
static bool      s_i2c_ok = true;

// ── Low-level I2C helpers (mirror the bmi08x style) ─────────────────────────
static bool i2cWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMP388_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  if (Wire.endTransmission() != 0) { s_i2c_ok = false; return false; }
  return true;
}

static bool i2cReadBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(BMP388_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    s_i2c_ok = false; memset(buf, 0, len); return false;
  }
  if (Wire.requestFrom(BMP388_I2C_ADDR, len) != len) {
    s_i2c_ok = false; memset(buf, 0, len); return false;
  }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Read 21 trim bytes from NVM and scale per Bosch BMP3 driver.
static bool bmp388ReadTrim() {
  uint8_t b[21];
  if (!i2cReadBurst(REG_NVM_PAR, b, 21)) return false;

  const uint16_t nvm_t1  = (uint16_t)(b[0]  | (b[1]  << 8));
  const uint16_t nvm_t2  = (uint16_t)(b[2]  | (b[3]  << 8));
  const int8_t   nvm_t3  = (int8_t)   b[4];
  const int16_t  nvm_p1  = (int16_t) (b[5]  | (b[6]  << 8));
  const int16_t  nvm_p2  = (int16_t) (b[7]  | (b[8]  << 8));
  const int8_t   nvm_p3  = (int8_t)   b[9];
  const int8_t   nvm_p4  = (int8_t)   b[10];
  const uint16_t nvm_p5  = (uint16_t)(b[11] | (b[12] << 8));
  const uint16_t nvm_p6  = (uint16_t)(b[13] | (b[14] << 8));
  const int8_t   nvm_p7  = (int8_t)   b[15];
  const int8_t   nvm_p8  = (int8_t)   b[16];
  const int16_t  nvm_p9  = (int16_t) (b[17] | (b[18] << 8));
  const int8_t   nvm_p10 = (int8_t)   b[19];
  const int8_t   nvm_p11 = (int8_t)   b[20];

  // Scale factors taken straight from the BMP3 reference driver.
  s_q.par_t1  = (float)nvm_t1  * 256.0f;                         // / 2^-8
  s_q.par_t2  = (float)nvm_t2  / 1073741824.0f;                  // / 2^30
  s_q.par_t3  = (float)nvm_t3  / 281474976710656.0f;             // / 2^48
  s_q.par_p1  = ((float)nvm_p1 - 16384.0f) / 1048576.0f;         // (x-2^14)/2^20
  s_q.par_p2  = ((float)nvm_p2 - 16384.0f) / 536870912.0f;       // (x-2^14)/2^29
  s_q.par_p3  = (float)nvm_p3  / 4294967296.0f;                  // / 2^32
  s_q.par_p4  = (float)nvm_p4  / 137438953472.0f;                // / 2^37
  s_q.par_p5  = (float)nvm_p5  * 8.0f;                           // / 2^-3
  s_q.par_p6  = (float)nvm_p6  / 64.0f;                          // / 2^6
  s_q.par_p7  = (float)nvm_p7  / 256.0f;                         // / 2^8
  s_q.par_p8  = (float)nvm_p8  / 32768.0f;                       // / 2^15
  s_q.par_p9  = (float)nvm_p9  / 281474976710656.0f;             // / 2^48
  s_q.par_p10 = (float)nvm_p10 / 281474976710656.0f;             // / 2^48
  s_q.par_p11 = (float)nvm_p11 / 36893488147419103232.0f;        // / 2^65
  return true;
}

static float bmp388CompensateTemp(uint32_t raw) {
  const float pd1 = (float)raw - s_q.par_t1;
  const float pd2 = pd1 * s_q.par_t2;
  s_q.t_lin = pd2 + (pd1 * pd1) * s_q.par_t3;
  return s_q.t_lin;
}

static float bmp388CompensatePress(uint32_t raw) {
  const float t  = s_q.t_lin;
  const float t2 = t * t;
  const float t3 = t2 * t;

  const float po1 = s_q.par_p5
                  + s_q.par_p6 * t
                  + s_q.par_p7 * t2
                  + s_q.par_p8 * t3;

  const float rp = (float)raw;
  const float po2 = rp * (s_q.par_p1
                          + s_q.par_p2 * t
                          + s_q.par_p3 * t2
                          + s_q.par_p4 * t3);

  const float rp2 = rp * rp;
  const float pd3 = rp2 * (s_q.par_p9 + s_q.par_p10 * t);
  const float pd4 = pd3 + (rp2 * rp) * s_q.par_p11;

  return po1 + po2 + pd4;
}

static bool bmp388ApplyConfig() {
  // Soft reset, then wait for the chip to finish (datasheet: ~2 ms).
  if (!i2cWriteReg(REG_CMD, CMD_SOFT_RESET)) return false;
  delay(10);

  if (!bmp388ReadTrim())                 return false;
  if (!i2cWriteReg(REG_OSR,    OSR_VAL)) return false;
  if (!i2cWriteReg(REG_ODR,    ODR_VAL)) return false;
  if (!i2cWriteReg(REG_CONFIG, CONFIG_VAL)) return false;
  // Power-control last: enables press+temp and switches to NORMAL mode.
  if (!i2cWriteReg(REG_PWR_CTRL, PWR_CTRL_VAL)) return false;
  delay(10);
  return true;
}

bool bmp388Init() {
  // Wire.begin() / setClock() done by the caller (shared with the IMU).
  uint8_t id = 0;
  uint8_t buf[1];
  if (!i2cReadBurst(REG_CHIP_ID, buf, 1)) {
#if USB_PRINT_ENABLED
    Serial.println("BMP388 chip-id read failed");
#endif
    return false;
  }
  id = buf[0];
#if USB_PRINT_ENABLED
  Serial.print("BMP388 chip ID: 0x"); Serial.println(id, HEX);
#endif
  if (id != CHIP_ID_BMP388) {
#if USB_PRINT_ENABLED
    Serial.print("Expected 0x"); Serial.print(CHIP_ID_BMP388, HEX);
    Serial.println(" — BMP388 init failed");
#endif
    return false;
  }

  if (!bmp388ApplyConfig()) {
#if USB_PRINT_ENABLED
    Serial.println("BMP388 config failed");
#endif
    return false;
  }

  s_i2c_ok = true;
#if USB_PRINT_ENABLED
  Serial.println("BMP388 init OK");
#endif
  return true;
}

void bmp388Recover() {
#if USB_PRINT_ENABLED
  Serial.println("BMP388 / Wire I2C error — recovering...");
#endif

  Wire.end();
  pinMode(BMP388_SCL_PIN, OUTPUT);
  for (int i = 0; i < 16; i++) {
    digitalWriteFast(BMP388_SCL_PIN, LOW);  delayMicroseconds(5);
    digitalWriteFast(BMP388_SCL_PIN, HIGH); delayMicroseconds(5);
  }
  pinMode(BMP388_SCL_PIN, INPUT);

  Wire.begin();
  Wire.setClock(BMP388_I2C_CLOCK_HZ);
  delay(10);

  if (bmp388ApplyConfig()) {
#if USB_PRINT_ENABLED
    Serial.println("BMP388 recovery OK");
#endif
  } else {
#if USB_PRINT_ENABLED
    Serial.println("BMP388 recovery FAILED");
#endif
  }
}

bool bmp388Read(float& temp_c, float& press_pa) {
  uint8_t b[6];
  if (!i2cReadBurst(REG_DATA_0, b, 6)) {
    bmp388Recover();
    return false;
  }

  const uint32_t raw_press = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
  const uint32_t raw_temp  = (uint32_t)b[3] | ((uint32_t)b[4] << 8) | ((uint32_t)b[5] << 16);

  temp_c   = bmp388CompensateTemp(raw_temp);
  press_pa = bmp388CompensatePress(raw_press);
  return true;
}
