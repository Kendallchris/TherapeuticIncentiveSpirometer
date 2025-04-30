#include "HomeScreen.h"
#include <lvgl.h>
#include "DataLogger.h"

extern DataLogger dataLogger;  // Use the global defined in .ino

HomeScreen::HomeScreen(TFT_eSPI &display)
  : tft(display) {
  // Initialization of the display instance if needed
}

void HomeScreen::show() {
  int currentHour = dataLogger.getCurrentHourMeasurements(true);

  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  // -- START Button (Green, Top Left) --
  lv_obj_t *start_btn = lv_btn_create(screen);
  lv_obj_set_size(start_btn, 200, 40);
  lv_obj_align(start_btn, LV_ALIGN_TOP_MID, 0, 10);
  lv_obj_add_event_cb(
    start_btn, [](lv_event_t *e) {
      digitalWrite(2, LOW);
      delay(50);
      digitalWrite(2, HIGH);
    },
    LV_EVENT_CLICKED, NULL);
  lv_obj_t *start_label = lv_label_create(start_btn);
  lv_label_set_text(start_label, "Start");
  lv_obj_align(start_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);

  // -- RESET Button (Red, Just Below Start) --
  lv_obj_t *reset_btn = lv_btn_create(screen);
  lv_obj_set_size(reset_btn, 200, 40);
  lv_obj_align(reset_btn, LV_ALIGN_TOP_MID, 0, 60);
  lv_obj_add_event_cb(
    reset_btn, [](lv_event_t *e) {
      digitalWrite(3, LOW);
      delay(50);
      digitalWrite(3, HIGH);
    },
    LV_EVENT_CLICKED, NULL);
  lv_obj_t *reset_label = lv_label_create(reset_btn);
  lv_label_set_text(reset_label, "Reset");
  lv_obj_align(reset_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(reset_btn, lv_color_hex(0xFF4C4C), LV_PART_MAIN);

  // -- Instruction Label (Centered Under Buttons) --
  lv_obj_t *instruction_label = lv_label_create(screen);
  lv_label_set_long_mode(instruction_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(instruction_label, 200);
  lv_label_set_text(instruction_label, "Keep it up! Get Those Measurements in!");
  lv_obj_set_style_text_align(instruction_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(instruction_label, LV_ALIGN_TOP_MID, 0, 140);

  // -- Count Box (BOTTOM of screen) --
  lv_obj_t *count_box = lv_obj_create(screen);
  lv_obj_set_size(count_box, 150, 80);
  lv_obj_align(count_box, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_bg_color(count_box, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_border_color(count_box, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_border_width(count_box, 3, LV_PART_MAIN);
  lv_obj_set_style_radius(count_box, 8, LV_PART_MAIN);

  lv_obj_t *count_label = lv_label_create(count_box);
  lv_label_set_text_fmt(count_label, "%d/10", currentHour);
  lv_obj_align(count_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_font(count_label, &lv_font_montserrat_28, LV_PART_MAIN);

  lv_refr_now(NULL);
}

