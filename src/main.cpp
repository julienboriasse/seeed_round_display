#include <Arduino.h>
// If build flags are missing, default to TFT_eSPI backend.
#ifndef USE_TFT_ESPI_LIBRARY
#define USE_TFT_ESPI_LIBRARY
#endif
#include <lvgl.h>
#include "lv_xiao_round_screen.h"

void setup() {
  Serial.begin(115200);
  delay(50);

  // 0–3 rotations; 0 keeps the FFC at the bottom.
  screen_rotation = 0;

  lv_init();
  lv_xiao_disp_init();
  lv_xiao_touch_init(); // Safe even if touch isn't used

  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello, World!");
  lv_obj_center(label);
}

void loop() {
  lv_timer_handler();
  delay(5);
}
