#include <Arduino.h>
// If build flags are missing, default to TFT_eSPI backend.
#ifndef USE_TFT_ESPI_LIBRARY
#define USE_TFT_ESPI_LIBRARY
#endif
#include <lvgl.h>
#include "lv_xiao_round_screen.h"
#include "display_ui.h"
#include "touch_utils.h"

uint8_t screen_rotation = 0;

static bool last_irq_state = false;

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println("Boot: Touchscreen test ready");

  // 0–3 rotations; 0 keeps the FFC at the bottom.
  screen_rotation = 0;

  lv_init();
  lv_xiao_disp_init();

  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  Wire.begin(SDA, SCL); // explicit pins for XIAO ESP32S3

  init_touch_test_ui();
  i2c_scan_once();
}

void loop() {
  static uint8_t last_x = 0, last_y = 0;
  static bool has_point = false;
  static uint32_t last_dump_ms = 0;

  uint8_t x = 0, y = 0;
  bool pressed = chsc6x_read_point(x, y);

  if (pressed && (!has_point || x != last_x || y != last_y)) {
    has_point = true;
    last_x = x;
    last_y = y;
    Serial.printf("[touch] Touch at (%u, %u)\n", x, y);

    lv_point_t p = {(lv_coord_t)x, (lv_coord_t)y};
    update_touch_point(p, "Touch detected");

    uint32_t now = millis();
    if (now - last_dump_ms > 250) {
      dump_touch_registers(TOUCH_ADDR_PRIMARY, 16);
      last_dump_ms = now;
    }
  } else if (!pressed && has_point) {
    has_point = false;
    Serial.println("[touch] Touch released");
    set_touch_idle_state();
  }

  bool irq_low = digitalRead(TOUCH_INT_PIN) == LOW;
  if (irq_low != last_irq_state) {
    //Serial.printf("[touch] IRQ line state changed: %s\n", irq_low ? "LOW" : "HIGH");
    last_irq_state = irq_low;
  }

  lv_timer_handler();
  delay(5);
}
