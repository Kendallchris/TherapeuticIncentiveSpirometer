#include "HomeScreen.h"
#include <lvgl.h>

HomeScreen::HomeScreen(TFT_eSPI &display) : tft(display) {
    // Initialization of the display instance if needed
}

void HomeScreen::show(int currentHour, int previousHour) {
    // Create a new screen
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_scr_load(screen);

    // Add labels for the instructions
    lv_obj_t *instruction_label = lv_label_create(screen);
    lv_label_set_text(instruction_label, "Press Green button\nto begin measurement");
    lv_obj_align(instruction_label, LV_ALIGN_TOP_MID, 0, 20);

    // Add labels for the measurement counts
    lv_obj_t *current_label = lv_label_create(screen);
    lv_label_set_text_fmt(current_label, "Current Hour: %d/10", currentHour);
    lv_obj_align(current_label, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *previous_label = lv_label_create(screen);
    lv_label_set_text_fmt(previous_label, "Previous Hour: %d/10", previousHour);
    lv_obj_align(previous_label, LV_ALIGN_CENTER, 0, 20);
}
