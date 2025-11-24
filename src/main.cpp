#include <Arduino.h>
// If build flags are missing, default to TFT_eSPI backend.
#ifndef USE_TFT_ESPI_LIBRARY
#define USE_TFT_ESPI_LIBRARY
#endif
#include <lvgl.h>
#include "lv_xiao_round_screen.h"

static constexpr uint8_t TOUCH_ADDR_PRIMARY = CHSC6X_I2C_ID;
static constexpr uint8_t TOUCH_ADDR_FALLBACK = CHSC6X_I2C_FALLBACK_ID;
static constexpr uint8_t TOUCH_INT_PIN = TOUCH_INT;
static constexpr uint8_t SCREEN_W = SCREEN_WIDTH;
static constexpr uint8_t SCREEN_H = SCREEN_HEIGHT;

uint8_t screen_rotation = 0;

static lv_obj_t *coord_label = nullptr;
static lv_obj_t *state_label = nullptr;
static lv_obj_t *touch_dot = nullptr;
static bool last_irq_state = false;

static bool chsc6x_read_point(uint8_t &x, uint8_t &y) {
  if (digitalRead(TOUCH_INT_PIN) != LOW) return false;

  uint8_t buf[5] = {0};
  uint8_t addr_used = TOUCH_ADDR_PRIMARY;
  auto read_packet = [&](uint8_t addr) -> uint8_t {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)0x00); // reset register pointer
    uint8_t tx_err = Wire.endTransmission(false);
    if (tx_err != 0) {
      Serial.printf("[touch] I2C addr=0x%02X set-pointer err=%u\n", addr, tx_err);
      return 0;
    }
    return Wire.requestFrom(addr, (uint8_t)5);
  };

  uint8_t len = read_packet(addr_used);
  if (len != 5 && TOUCH_ADDR_FALLBACK != addr_used) {
    addr_used = TOUCH_ADDR_FALLBACK;
    len = read_packet(addr_used);
  }

  if (len != 5) {
    Serial.printf("[touch] I2C read failed: got %u bytes\n", len);
    return false;
  }

  Wire.readBytes(buf, 5);
  //Serial.printf("[touch] Raw addr=0x%02X: %02x %02x %02x %02x %02x\n",
  //              addr_used, buf[0], buf[1], buf[2], buf[3], buf[4]);

  if (buf[0] != 0x01) return false; // header not indicating touch
  x = buf[2];
  y = buf[4];
  chsc6x_convert_xy(&x, &y);
  return true;
}

static void dump_touch_registers(uint8_t addr, uint8_t count) {
  uint8_t buf[32] = {0};
  count = min<uint8_t>(count, sizeof(buf));

  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0x00);
  uint8_t tx_err = Wire.endTransmission(false);
  if (tx_err != 0) {
    //Serial.printf("[touch] Dump addr=0x%02X set-pointer err=%u\n", addr, tx_err);
    return;
  }

  uint8_t len = Wire.requestFrom(addr, count);
  if (len != count) {
    //Serial.printf("[touch] Dump addr=0x%02X got %u/%u bytes\n", addr, len, count);
    return;
  }
  Wire.readBytes(buf, len);
  //Serial.print("[touch] Dump addr=0x");
  //Serial.print(addr, HEX);
  //Serial.print(" :");
  //for (uint8_t i = 0; i < len; i++) {
  //  Serial.printf(" %02x", buf[i]);
  //}
  Serial.println();
}

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
  lv_label_set_text(hint, "Raw CHSC6X read + LVGL render.");
  lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 90);

  touch_dot = lv_obj_create(screen);
  lv_obj_set_size(touch_dot, 12, 12);
  lv_obj_set_style_radius(touch_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(touch_dot, lv_color_hex(0xff6600), 0);
  lv_obj_set_style_border_width(touch_dot, 0, 0);

  set_idle_state();
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

  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  Wire.begin(SDA, SCL); // explicit pins for XIAO ESP32S3

  build_touch_test_ui();
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
    set_idle_state();
  }

  bool irq_low = digitalRead(TOUCH_INT_PIN) == LOW;
  if (irq_low != last_irq_state) {
    //Serial.printf("[touch] IRQ line state changed: %s\n", irq_low ? "LOW" : "HIGH");
    last_irq_state = irq_low;
  }

  lv_timer_handler();
  delay(5);
}
