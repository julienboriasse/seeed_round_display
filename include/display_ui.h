#pragma once

#include <lvgl.h>

void init_touch_test_ui();
void set_touch_idle_state();
void update_touch_point(const lv_point_t &point, const char *state_text);
