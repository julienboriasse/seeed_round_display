#include "touch_utils.h"

#include <Wire.h>

static bool chsc6x_read_raw(uint8_t addr, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0x00); // reset register pointer
  uint8_t tx_err = Wire.endTransmission(false);
  if (tx_err != 0) {
    Serial.printf("[touch] I2C addr=0x%02X set-pointer err=%u\n", addr, tx_err);
    return false;
  }

  uint8_t got = Wire.requestFrom(addr, (uint8_t)len);
  if (got != len) {
    Serial.printf("[touch] I2C addr=0x%02X read %u/%u bytes\n", addr, got, (uint8_t)len);
    return false;
  }

  Wire.readBytes(buf, len);
  return true;
}

bool chsc6x_read_point(uint8_t &x, uint8_t &y) {
  if (digitalRead(TOUCH_INT_PIN) != LOW) return false;

  uint8_t buf[CHSC6X_READ_POINT_LEN] = {0};
  uint8_t addr_used = TOUCH_ADDR_PRIMARY;
  bool ok = chsc6x_read_raw(addr_used, buf, sizeof(buf));

  if (!ok && TOUCH_ADDR_FALLBACK != addr_used) {
    addr_used = TOUCH_ADDR_FALLBACK;
    ok = chsc6x_read_raw(addr_used, buf, sizeof(buf));
  }

  if (!ok) return false;
  if (buf[0] != 0x01) return false; // header not indicating touch

  x = buf[2];
  y = buf[4];
  chsc6x_convert_xy(&x, &y);
  return true;
}

void dump_touch_registers(uint8_t addr, uint8_t count) {
  uint8_t buf[32] = {0};
  count = min<uint8_t>(count, sizeof(buf));

  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0x00);
  uint8_t tx_err = Wire.endTransmission(false);
  if (tx_err != 0) {
    return;
  }

  uint8_t len = Wire.requestFrom(addr, count);
  if (len != count) {
    return;
  }
  Wire.readBytes(buf, len);
  Serial.println();
}

void i2c_scan_once() {
  Serial.println("[i2c] Scanning bus on Wire (SDA, SCL)...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("[i2c] Found device at 0x%02X\n", addr);
    }
    delay(2);
  }
  Serial.println("[i2c] Scan complete");
}
