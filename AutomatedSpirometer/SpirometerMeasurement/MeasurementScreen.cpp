#include "MeasurementScreen.h"
#include <lvgl.h>

MeasurementScreen::MeasurementScreen(TFT_eSPI &display)
  : tft(display), countdown_label(nullptr) {
  // Constructor initialization
}

void MeasurementScreen::showWaitingWithCountdown() {
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);  // Match screen size
    lv_scr_load(screen);

    countdown_label = lv_label_create(screen);
    lv_label_set_text(countdown_label, "Beginning measurement...");
    lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 0);

    countdown_duration = 10;      // Set countdown duration in seconds
    countdown_active = true;     // Enable countdown state
    countdown_start = millis();  // Start countdown
    lv_refr_now(NULL);           // Force immediate refresh
}

void MeasurementScreen::updateCountdown() {
    if (countdown_label) {
        int elapsed = (millis() - countdown_start) / 1000;
        int remaining = countdown_duration - elapsed;

        if (remaining > 0) {
            // Update the countdown text
            lv_label_set_text_fmt(countdown_label, "Beginning measurement in %d seconds\nPress green button to cancel", remaining);
            lv_refr_now(NULL);  // Force refresh after updating countdown text
        } else {
            // Countdown complete, transition to measurement phase
            lv_label_set_text(countdown_label, "Waiting for object...");
            countdown_label = nullptr;  // Stop countdown updates
            countdown_active = false;  // Disable countdown state

            // Call a new function to handle the measurement phase transition
            beginMeasurementPhase();
        }
    }
}

void MeasurementScreen::beginMeasurementPhase() {
    // Create a full-screen container for the measurement phase
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);  // Match screen size
    lv_scr_load(screen);

    // Add a message for the measurement phase
    lv_obj_t *measurement_label = lv_label_create(screen);
    lv_label_set_text(measurement_label, "Waiting for object...\nPress green button to cancel");
    lv_obj_align(measurement_label, LV_ALIGN_CENTER, 0, 0);

    lv_refr_now(NULL);  // Ensure the new screen is fully updated
}

void MeasurementScreen::showSuccess() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  lv_obj_t *success_label = lv_label_create(screen);
  lv_label_set_text(success_label, "SUCCESS!");
  lv_obj_align(success_label, LV_ALIGN_CENTER, 0, 0);

  // Force the display to refresh and ensure the screen stays for 2 seconds
  lv_timer_create([](lv_timer_t *t) {
    lv_scr_load(lv_scr_act());  // Return to active screen
    lv_timer_del(t);            // Delete timer
  },
                  2000, NULL);  // 2-second delay
}

void MeasurementScreen::showNoObject() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  lv_obj_t *no_object_label = lv_label_create(screen);
  lv_label_set_text(no_object_label, "No object detected.");
  lv_obj_align(no_object_label, LV_ALIGN_CENTER, 0, 0);
}
