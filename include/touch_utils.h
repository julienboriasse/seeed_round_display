#pragma once

#include <Arduino.h>
#include "lv_xiao_round_screen.h"

static constexpr uint8_t TOUCH_ADDR_PRIMARY = CHSC6X_I2C_ID;
static constexpr uint8_t TOUCH_ADDR_FALLBACK = CHSC6X_I2C_FALLBACK_ID;
static constexpr uint8_t TOUCH_INT_PIN = TOUCH_INT;
static constexpr uint8_t SCREEN_W = SCREEN_WIDTH;
static constexpr uint8_t SCREEN_H = SCREEN_HEIGHT;

bool chsc6x_read_point(uint8_t &x, uint8_t &y);
void dump_touch_registers(uint8_t addr, uint8_t count);
void i2c_scan_once();
