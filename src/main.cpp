#include <Arduino.h>
// If build flags are missing, default to TFT_eSPI backend.
#ifndef USE_TFT_ESPI_LIBRARY
#define USE_TFT_ESPI_LIBRARY
#endif
#include <lvgl.h>
#include "lv_xiao_round_screen.h"

static lv_obj_t *coord_label = nullptr;
static lv_obj_t *state_label = nullptr;
static lv_obj_t *touch_dot = nullptr;
static bool last_irq_state = false;

static void set_idle_state() {
  lv_label_set_text(state_label, "Waiting for touch...");
  lv_label_set_text(coord_label, "X:---  Y:---");
  lv_obj_add_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
}

static void update_touch_point(const lv_point_t &point, const char *state_text) {
  lv_label_set_text_fmt(coord_label, "X:%3d  Y:%3d", point.x, point.y);
  lv_label_set_text(state_label, state_text);
  lv_obj_clear_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_pos(touch_dot,
                 point.x - (lv_obj_get_width(touch_dot) / 2),
                 point.y - (lv_obj_get_height(touch_dot) / 2));
}

static void touch_event_cb(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING &&
      code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) {
    return;
  }

  lv_indev_t *indev = lv_event_get_indev(event);
  lv_point_t point = {0, 0};
  bool has_point = indev != nullptr;
  if (has_point) {
    lv_indev_get_point(indev, &point);
  }

  switch (code) {
  case LV_EVENT_PRESSED:
    if (has_point) {
      Serial.printf("Touch pressed at (%d, %d)\n", point.x, point.y);
      update_touch_point(point, "Touch detected");
    }
    break;
  case LV_EVENT_PRESSING:
    if (has_point) {
      Serial.printf("Touch move at (%d, %d)\n", point.x, point.y);
      update_touch_point(point, "Moving...");
    }
    break;
  case LV_EVENT_RELEASED:
    if (has_point) {
      Serial.printf("Touch released at (%d, %d)\n", point.x, point.y);
      update_touch_point(point, "Released");
    }
    lv_obj_add_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
    break;
  case LV_EVENT_PRESS_LOST:
    Serial.println("Touch lost");
    set_idle_state();
    break;
  default:
    break;
  }
}

static void build_touch_test_ui() {
  lv_obj_t *screen = lv_scr_act();
  lv_obj_add_flag(screen, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t *title = lv_label_create(screen);
  lv_label_set_text(title, "Touchscreen test");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  state_label = lv_label_create(screen);
  lv_obj_align(state_label, LV_ALIGN_TOP_MID, 0, 40);

  coord_label = lv_label_create(screen);
  lv_obj_align(coord_label, LV_ALIGN_TOP_MID, 0, 60);

  lv_obj_t *hint = lv_label_create(screen);
  lv_label_set_text(hint, "Tap, drag, and release to verify input.");
  lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 90);

  touch_dot = lv_obj_create(screen);
  lv_obj_set_size(touch_dot, 12, 12);
  lv_obj_set_style_radius(touch_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(touch_dot, lv_color_hex(0xff6600), 0);
  lv_obj_set_style_border_width(touch_dot, 0, 0);

  set_idle_state();
  lv_obj_add_event_cb(screen, touch_event_cb, LV_EVENT_ALL, nullptr);
}

static void i2c_scan_once() {
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

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println("Boot: Touchscreen test ready");

  // 0–3 rotations; 0 keeps the FFC at the bottom.
  screen_rotation = 0;

  lv_init();
  lv_xiao_disp_init();
  lv_xiao_touch_init(); // Safe even if touch isn't used

  build_touch_test_ui();
  i2c_scan_once();
}

void loop() {
  bool irq_low = digitalRead(TOUCH_INT) == LOW;
  if (irq_low != last_irq_state) {
    Serial.printf("[touch] IRQ line state changed: %s\n", irq_low ? "LOW" : "HIGH");
    last_irq_state = irq_low;
  }

  lv_timer_handler();
  delay(5);
}
