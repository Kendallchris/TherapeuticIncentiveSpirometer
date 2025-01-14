#include "MeasurementScreen.h"
#include <lvgl.h>

MeasurementScreen::MeasurementScreen(TFT_eSPI &display)
  : tft(display), countdown_label(nullptr) {
  // Constructor initialization
}

void MeasurementScreen::showWaitingWithCountdown() {
  // Create and display a new screen for measurement
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);  // Load the new screen

  lv_obj_t *waiting_label = lv_label_create(screen);
  lv_label_set_text(waiting_label, "Waiting for object...\nPress green button to cancel");
  lv_obj_align(waiting_label, LV_ALIGN_TOP_MID, 0, 20);

  countdown_label = lv_label_create(screen);
  lv_label_set_text(countdown_label, "Beginning measurement...");
  lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 0);

  countdown_start = millis();  // Start countdown timer

  lv_refr_now(NULL);  // Force immediate refresh after loading the screen
}

void MeasurementScreen::updateCountdown() {
  if (countdown_label) {
    int elapsed = (millis() - countdown_start) / 1000;
    int remaining = countdown_duration - elapsed;

    if (remaining > 0) {
      lv_label_set_text_fmt(countdown_label, "Beginning measurement in %d seconds", remaining);
    } else {
      lv_label_set_text(countdown_label, "Measurement started!");
      countdown_label = nullptr;  // Stop further updates
    }
    lv_timer_handler();  // Ensure screen updates

    // Force refresh after updating countdown text
    lv_refr_now(NULL);
  }
}

void MeasurementScreen::showSuccess() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  lv_obj_t *success_label = lv_label_create(screen);
  lv_label_set_text(success_label, "SUCCESS!");
  lv_obj_align(success_label, LV_ALIGN_CENTER, 0, 0);

  delay(2000);  // Show success message for 2 seconds
}

void MeasurementScreen::showNoObject() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  lv_obj_t *no_object_label = lv_label_create(screen);
  lv_label_set_text(no_object_label, "No object detected.");
  lv_obj_align(no_object_label, LV_ALIGN_CENTER, 0, 0);
}
