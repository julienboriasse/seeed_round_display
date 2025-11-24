#include "display_ui.h"

static lv_obj_t *coord_label = nullptr;
static lv_obj_t *state_label = nullptr;
static lv_obj_t *touch_dot = nullptr;

void set_touch_idle_state() {
  lv_label_set_text(state_label, "Waiting for touch...");
  lv_label_set_text(coord_label, "X:---  Y:---");
  lv_obj_add_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
}

void update_touch_point(const lv_point_t &point, const char *state_text) {
  lv_label_set_text_fmt(coord_label, "X:%3d  Y:%3d", point.x, point.y);
  lv_label_set_text(state_label, state_text);
  lv_obj_clear_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_pos(touch_dot,
                 point.x - (lv_obj_get_width(touch_dot) / 2),
                 point.y - (lv_obj_get_height(touch_dot) / 2));
}

void init_touch_test_ui() {
  lv_obj_t *screen = lv_scr_act();
  lv_obj_add_flag(screen, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t *title = lv_label_create(screen);
  lv_label_set_text(title, "Touchscreen test");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  state_label = lv_label_create(screen);
  lv_obj_align(state_label, LV_ALIGN_CENTER, 0, -30);

  coord_label = lv_label_create(screen);
  lv_obj_align(coord_label, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t *hint = lv_label_create(screen);
  lv_label_set_text(hint, "Raw CHSC6X read + LVGL render.");
  lv_obj_align(hint, LV_ALIGN_CENTER, 0, 30);

  touch_dot = lv_obj_create(screen);
  lv_obj_set_size(touch_dot, 12, 12);
  lv_obj_set_style_radius(touch_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(touch_dot, lv_color_hex(0xff6600), 0);
  lv_obj_set_style_border_width(touch_dot, 0, 0);

  set_touch_idle_state();
}
