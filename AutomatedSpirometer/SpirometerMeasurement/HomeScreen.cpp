#include "HomeScreen.h"
#include <lvgl.h>

HomeScreen::HomeScreen(TFT_eSPI &display)
  : tft(display) {
  // Initialization of the display instance if needed
}

void HomeScreen::show(int currentHour) {
  // Create a new screen
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  // Add label for instructions
  lv_obj_t *instruction_label = lv_label_create(screen);
  lv_label_set_text(instruction_label, "Keep it up! Get Those Measurements in!");
  lv_obj_align(instruction_label, LV_ALIGN_TOP_MID, 0, 10);

  // **Measurement Count Display (Dominant Box)**
  lv_obj_t *count_box = lv_obj_create(screen);
  lv_obj_set_size(count_box, 120, 80);  // Box size
  lv_obj_align(count_box, LV_ALIGN_CENTER, 0, -40);

  // Apply Box Style
  lv_obj_set_style_bg_color(count_box, lv_color_hex(0xFFFFFF), LV_PART_MAIN);      // White background
  lv_obj_set_style_border_color(count_box, lv_color_hex(0x000000), LV_PART_MAIN);  // Black border
  lv_obj_set_style_border_width(count_box, 4, LV_PART_MAIN);                       // Border thickness
  lv_obj_set_style_radius(count_box, 10, LV_PART_MAIN);                            // Rounded corners

  // **Measurement Count Label (Large Font)**
  lv_obj_t *count_label = lv_label_create(count_box);
  lv_label_set_text_fmt(count_label, "%d/10", currentHour);  // Only show the number
  lv_obj_align(count_label, LV_ALIGN_CENTER, 0, 0);          // Center the text

  // Apply Large Font Style
  lv_obj_set_style_text_font(count_label, &lv_font_montserrat_28, LV_PART_MAIN);   // Larger font
  lv_obj_set_style_text_color(count_label, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text

  // **Start Measurement Button (Green)**
  lv_obj_t *start_btn = lv_btn_create(screen);
  lv_obj_set_size(start_btn, 130, 60);
  lv_obj_align(start_btn, LV_ALIGN_CENTER, -80, 75);  // Moved further down
  lv_obj_add_event_cb(
    start_btn, [](lv_event_t *e) {
      // Simulate pressing the green button to start measurement
      digitalWrite(2, LOW);  // GPIO2 for measurement mode button
      delay(50);
      digitalWrite(2, HIGH);
    },
    LV_EVENT_CLICKED, NULL);

  lv_obj_t *start_label = lv_label_create(start_btn);
  lv_label_set_text(start_label, "Start");
  lv_obj_align(start_label, LV_ALIGN_CENTER, 0, 0);  // Center text properly

  // Apply Green Style
  lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);    // Green background
  lv_obj_set_style_radius(start_btn, 12, LV_PART_MAIN);                          // Rounded corners
  lv_obj_set_style_text_color(start_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text

  // **Reset Data Button (Red)**
  lv_obj_t *reset_btn = lv_btn_create(screen);
  lv_obj_set_size(reset_btn, 130, 60);
  lv_obj_align(reset_btn, LV_ALIGN_CENTER, 80, 75);  // Moved further down
  lv_obj_add_event_cb(
    reset_btn, [](lv_event_t *e) {
      // Simulate pressing the red button to trigger the reset confirmation screen
      digitalWrite(3, LOW);  // GPIO3 for reset button
      delay(50);
      digitalWrite(3, HIGH);
    },
    LV_EVENT_CLICKED, NULL);

  lv_obj_t *reset_label = lv_label_create(reset_btn);
  lv_label_set_text(reset_label, "Reset Data");
  lv_obj_align(reset_label, LV_ALIGN_CENTER, 0, 0);  // Center text properly

  // Apply Red Style
  lv_obj_set_style_bg_color(reset_btn, lv_color_hex(0xFF4C4C), LV_PART_MAIN);    // Red background
  lv_obj_set_style_radius(reset_btn, 12, LV_PART_MAIN);                          // Rounded corners
  lv_obj_set_style_text_color(reset_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text

  lv_refr_now(NULL);  // Force screen refresh
}
